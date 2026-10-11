/*
 * Copyright © 2026 Neil Albrock
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

// The hardware-independent parts of USB audio streaming, kept apart from the driver glue in usb_audio.cpp so they can
// be unit tested.

#include "io/usb/usb_audio_descriptor.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

namespace deluge::io::usb::audio {

struct Frame {
	int32_t l;
	int32_t r;
};

// Single-producer single-consumer ring of stereo frames, safe between the audio routine and the USB ISR: each index
// is only ever written by one side. Capacity must be a power of two.
//
// Both sides run on the same (only) core, one of them in an interrupt, so the CPU always sees memory in program order
// and only the compiler needs keeping in check - hence signal fences rather than acquire/release, which on ARMv7 would
// cost a full memory barrier per access.
template <size_t kCapacity>
class FrameRing {
	static_assert((kCapacity & (kCapacity - 1)) == 0, "capacity must be a power of two");

public:
	size_t size() const {
		size_t size = writeIndex_.load(std::memory_order_relaxed) - readIndex_.load(std::memory_order_relaxed);
		std::atomic_signal_fence(std::memory_order_acquire);
		return size;
	}
	size_t space() const { return kCapacity - size(); }

	// Producer side. Returns how many frames fitted - any more are dropped.
	size_t push(std::span<const Frame> frames) {
		size_t write = writeIndex_.load(std::memory_order_relaxed);
		size_t count = std::min(frames.size(), kCapacity - (write - readIndex_.load(std::memory_order_relaxed)));
		std::atomic_signal_fence(std::memory_order_acquire); // Don't overwrite frames before seeing they're read
		for (size_t i = 0; i < count; i++) {
			buffer_[(write + i) & (kCapacity - 1)] = frames[i];
		}
		std::atomic_signal_fence(std::memory_order_release); // Frames are written before the index says so
		writeIndex_.store(write + count, std::memory_order_relaxed);
		return count;
	}

	// Consumer side. Returns how many frames were available, up to dest.size().
	size_t pop(std::span<Frame> dest) {
		size_t read = readIndex_.load(std::memory_order_relaxed);
		size_t count = std::min(dest.size(), writeIndex_.load(std::memory_order_relaxed) - read);
		std::atomic_signal_fence(std::memory_order_acquire); // Don't read frames before seeing they're written
		for (size_t i = 0; i < count; i++) {
			dest[i] = buffer_[(read + i) & (kCapacity - 1)];
		}
		std::atomic_signal_fence(std::memory_order_release); // Frames are read before the index frees them
		readIndex_.store(read + count, std::memory_order_relaxed);
		return count;
	}

	// Consumer side
	void discard(size_t count) {
		size_t read = readIndex_.load(std::memory_order_relaxed);
		count = std::min(count, writeIndex_.load(std::memory_order_relaxed) - read);
		readIndex_.store(read + count, std::memory_order_relaxed);
	}

	// Consumer side. Only safe to call while the producer isn't pushing.
	void clear() { readIndex_.store(writeIndex_.load(std::memory_order_relaxed), std::memory_order_relaxed); }

private:
	std::array<Frame, kCapacity> buffer_{};
	std::atomic<size_t> writeIndex_{0};
	std::atomic<size_t> readIndex_{0};
};

// Decides how many frames go in each packet to the computer. The stream is synchronous - locked to USB's 1ms frames -
// so this is a fixed pattern averaging 44.1: nine packets of 44, then one of 45. That's within one frame of the
// nominal size, as Windows requires (FMT-2 2.3.1.1).
class PacketSizer {
	static_assert(USB_AUDIO_SAMPLE_RATE % 100 == 0, "the pattern repeats every 10 packets");
	static constexpr int32_t kPacketsPerCycle = 10;
	static constexpr int32_t kFramesPerCycle = USB_AUDIO_SAMPLE_RATE / 100;

public:
	static constexpr int32_t kMaxFrames = kFramesPerCycle / kPacketsPerCycle + 1;

	void reset() { remainder_ = 0; }

	int32_t nextPacketFrames() {
		remainder_ += kFramesPerCycle;
		int32_t frames = remainder_ / kPacketsPerCycle;
		remainder_ -= frames * kPacketsPerCycle;
		return frames;
	}

private:
	int32_t remainder_ = 0;
};

// Packs frames as the 3-byte little-endian samples the computer expects. Samples are 32-bit with the 24 meaningful
// bits at the top, as they go to the codec. Returns the number of bytes written.
inline size_t packFrames24(std::span<const Frame> frames, uint8_t* dest) {
	uint8_t* out = dest;
	for (const Frame& frame : frames) {
		for (int32_t sample : {frame.l, frame.r}) {
			uint32_t bits = static_cast<uint32_t>(sample);
			*out++ = (bits >> 8) & 0xFF;
			*out++ = (bits >> 16) & 0xFF;
			*out++ = (bits >> 24) & 0xFF;
		}
	}
	return out - dest;
}

// Keeps the stream to the computer at a steady latency. The audio routine renders ahead in bursts, so the ring's fill
// level wobbles; this only steps in when it has gone well outside that wobble - e.g. after the computer stopped
// collecting packets for a while - by skipping frames down to the target.
template <size_t kTargetFrames, size_t kSlackFrames>
size_t framesToSkipForLatency(size_t framesQueued) {
	if (framesQueued > kTargetFrames + kSlackFrames) {
		return framesQueued - kTargetFrames;
	}
	return 0;
}

// Unpacks 3-byte little-endian samples from the computer into 32-bit ones with the 24 meaningful bits at the top, as
// the codec gives them. Any partial frame at the end is ignored. Returns the number of frames.
inline size_t unpackFrames24(const uint8_t* src, size_t numBytes, Frame* dest, size_t maxFrames) {
	auto readSample = [&src]() {
		uint32_t bits = (uint32_t{src[0]} << 8) | (uint32_t{src[1]} << 16) | (uint32_t{src[2]} << 24);
		src += 3;
		return static_cast<int32_t>(bits);
	};

	size_t numFrames = std::min(numBytes / USB_AUDIO_BYTES_PER_FRAME, maxFrames);
	for (size_t i = 0; i < numFrames; i++) {
		dest[i].l = readSample();
		dest[i].r = readSample();
	}
	return numFrames;
}

/*********************************************************************************************************************
 * Clock drift
 *
 * Both streams run on the computer's clock, which drifts against the codec's crystal by up to a few hundred ppm.
 * DriftResampler converts between the two, and RateServo steers its ratio so that the ring on the computer's side
 * stays at a steady fill level - which only happens when the ratio matches the drift. Computer -> Deluge resamples
 * from the computer's clock to the codec's, and Deluge -> computer the other way.
 *********************************************************************************************************************/

// Resamples by a ratio very close to 1, with 4-point cubic (Catmull-Rom) interpolation.
class DriftResampler {
public:
	static constexpr uint64_t kUnity = uint64_t{1} << 32; // Increment of 1 source frame per output frame

	void reset() {
		history_ = {};
		phase_ = 0;
	}

	// Source frames to advance per output frame, as a 32.32 fixed-point number
	void setIncrement(uint64_t increment) { increment_ = increment; }

	// How many frames render() can produce from numSourceFrames more source frames, without running dry
	size_t framesAvailable(size_t numSourceFrames) const {
		// Output frame k takes the source up to frame (phase_ + k * increment_) >> 32, which must be one we'll have
		uint64_t limit = (uint64_t{numSourceFrames} + 1) * kUnity;
		if (phase_ >= limit) {
			return 0;
		}
		return static_cast<size_t>((limit - phase_ - 1) / increment_ + 1);
	}

	// Fills out, taking source frames from the ring as needed. Returns false if the ring ran dry, in which case
	// silence was used in place of the missing frames.
	template <size_t kCapacity>
	bool render(std::span<Frame> out, FrameRing<kCapacity>& source) {
		bool ranDry = false;
		for (Frame& frame : out) {
			while (phase_ >= kUnity) {
				Frame next{0, 0};
				if (source.pop({&next, 1}) == 0) {
					ranDry = true;
				}
				history_ = {history_[1], history_[2], history_[3], next};
				phase_ -= kUnity;
			}
			uint32_t t = static_cast<uint32_t>(phase_ >> 8); // Q24
			frame.l = interpolate(history_[0].l, history_[1].l, history_[2].l, history_[3].l, t);
			frame.r = interpolate(history_[0].r, history_[1].r, history_[2].r, history_[3].r, t);
			phase_ += increment_;
		}
		return !ranDry;
	}

private:
	// Between x1 and x2, t of the way (Q24). Works on the 24 meaningful bits, so nothing overflows 64 bits.
	static int32_t interpolate(int32_t x0, int32_t x1, int32_t x2, int32_t x3, uint32_t t) {
		int64_t a = x0 >> 8;
		int64_t b = x1 >> 8;
		int64_t c = x2 >> 8;
		int64_t d = x3 >> 8;
		// Twice the usual Catmull-Rom coefficients, to stay in integers
		int64_t k1 = c - a;
		int64_t k2 = 2 * a - 5 * b + 4 * c - d;
		int64_t k3 = (d - a) + 3 * (b - c);
		int64_t y = ((((k3 * t) >> 24) + k2) * t) >> 24;
		y = (((y + k1) * t) >> 24);
		y = b + (y >> 1);
		y = std::clamp<int64_t>(y, -(int64_t{1} << 23), (int64_t{1} << 23) - 1);
		return static_cast<int32_t>(y << 8);
	}

	// The output lies between history_[1] and history_[2], phase_ (32.32) of the way
	std::array<Frame, 4> history_{};
	uint64_t phase_ = 0;
	uint64_t increment_ = kUnity;
};

// Proportional-integral control of the resampling ratio from the fill level of the ring on the computer's side. The
// fill level is smoothed first, as the USB side moves a packet every 1ms but the audio routine moves blocks of up to
// SSI_TX_BUFFER_NUM_SAMPLES.
class RateServo {
public:
	static constexpr float kSampleRate = USB_AUDIO_SAMPLE_RATE;
	static constexpr float kFillSmoothingSeconds = 0.25;
	static constexpr float kProportionalPpmPerFrame = 5; // Settles in a few seconds
	static constexpr float kIntegralSeconds = 20;        // Takes over the steady drift, leaving no fill error
	static constexpr float kMaxCorrectionPpm = 1000;     // Far beyond any real crystal's error

	explicit RateServo(float targetFill) : targetFill_(targetFill) {}

	// Call when (re)starting, with the fill level then. Keeps the drift learned so far unless forgetDrift.
	void reset(size_t fill, bool forgetDrift) {
		smoothedFill_ = static_cast<float>(fill);
		if (forgetDrift) {
			integralPpm_ = 0;
		}
	}

	// Call once per block, with the ring's fill level and the number of frames just rendered. Returns the increment
	// for the resampler.
	uint64_t update(size_t fill, size_t framesRendered) {
		float weight = std::min(1.f, framesRendered / (kFillSmoothingSeconds * kSampleRate));
		smoothedFill_ += (static_cast<float>(fill) - smoothedFill_) * weight;

		float error = smoothedFill_ - targetFill_; // More queued than wanted: take more source frames per output frame
		integralPpm_ += error * kProportionalPpmPerFrame * framesRendered / (kIntegralSeconds * kSampleRate);
		integralPpm_ = std::clamp(integralPpm_, -kMaxCorrectionPpm, kMaxCorrectionPpm);

		correctionPpm_ =
		    std::clamp(error * kProportionalPpmPerFrame + integralPpm_, -kMaxCorrectionPpm, kMaxCorrectionPpm);
		return static_cast<uint64_t>(static_cast<double>(DriftResampler::kUnity) * (1.0 + correctionPpm_ * 1e-6));
	}

	float correctionPpm() const { return correctionPpm_; }

private:
	float targetFill_;
	float smoothedFill_ = 0;
	float integralPpm_ = 0;
	float correctionPpm_ = 0;
};

} // namespace deluge::io::usb::audio

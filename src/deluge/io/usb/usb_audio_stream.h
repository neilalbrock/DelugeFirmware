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

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

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
		size_t start = write & (kCapacity - 1);
		size_t first = std::min(count, kCapacity - start); // Up to the end of the buffer, then the rest from the start
		std::copy_n(frames.begin(), first, &buffer_[start]);
		std::copy_n(frames.begin() + first, count - first, &buffer_[0]);
		std::atomic_signal_fence(std::memory_order_release); // Frames are written before the index says so
		writeIndex_.store(write + count, std::memory_order_relaxed);
		return count;
	}

	// Consumer side. Returns how many frames were available, up to dest.size().
	size_t pop(std::span<Frame> dest) {
		size_t read = readIndex_.load(std::memory_order_relaxed);
		size_t count = std::min(dest.size(), writeIndex_.load(std::memory_order_relaxed) - read);
		std::atomic_signal_fence(std::memory_order_acquire); // Don't read frames before seeing they're written
		size_t start = read & (kCapacity - 1);
		size_t first = std::min(count, kCapacity - start); // Up to the end of the buffer, then the rest from the start
		std::copy_n(&buffer_[start], first, dest.begin());
		std::copy_n(&buffer_[0], count - first, dest.begin() + first);
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

// How many frames of the codec's input buffer (kBufferFrames long) to process now, from processedUpTo on. Only what's
// captured can be processed - up to capturedUpTo - and what a render needs has to be - up to neededUpTo. If neededUpTo
// is already processed, that's nothing; if not, it's up to batchFrames beyond it, as far as is captured. Positions are
// frames from the buffer's start. A neededUpTo beyond capturedUpTo means everything that's captured.
template <size_t kBufferFrames>
size_t inputFramesToProcess(uint32_t processedUpTo, uint32_t capturedUpTo, uint32_t neededUpTo, size_t batchFrames) {
	static_assert((kBufferFrames & (kBufferFrames - 1)) == 0, "positions wrap with a mask");
	constexpr uint32_t kMask = kBufferFrames - 1;
	size_t backlog = (capturedUpTo - processedUpTo) & kMask; // Captured but not processed
	size_t spare = (capturedUpTo - neededUpTo) & kMask;      // Captured beyond what's needed
	if (spare > kBufferFrames / 2) {
		spare = 0; // Needed reaches beyond what's captured
	}
	if (backlog <= spare) {
		return 0;
	}
	return backlog - (spare > batchFrames ? spare - batchFrames : 0);
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

namespace catmull_rom {

// Between x1 and x2, t of the way (Q31). Catmull-Rom, with twice the usual coefficients to stay in integers and
// evaluated by Horner's rule, where each multiply by t is a saturating doubling multiply returning the high half
// (vqdmulh) - so it's by t in [0, 1) - and the last one is halved back to the usual coefficients.
// Works on the samples' top 26 bits: the samples' 24 meaningful bits plus 2 for precision. Then no step can overflow -
// k2, the biggest coefficient, is at most 12 * 2^25, and each partial sum stays under 2^30. The result is shifted back
// up with saturation, which clips any overshoot.
#if defined(__ARM_NEON)
[[gnu::always_inline]] inline int32x4_t interpolate(int32x4_t x0, int32x4_t x1, int32x4_t x2, int32x4_t x3,
                                                    int32x4_t t) {
	int32x4_t a = vshrq_n_s32(x0, 6);
	int32x4_t b = vshrq_n_s32(x1, 6);
	int32x4_t c = vshrq_n_s32(x2, 6);
	int32x4_t d = vshrq_n_s32(x3, 6);
	int32x4_t k1 = vsubq_s32(c, a);
	int32x4_t k2 = vsubq_s32(vaddq_s32(vmlsq_n_s32(vshlq_n_s32(a, 1), b, 5), vshlq_n_s32(c, 2)), d); // 2a-5b+4c-d
	int32x4_t k3 = vmlaq_n_s32(vsubq_s32(d, a), vsubq_s32(b, c), 3);                                 // (d-a)+3(b-c)
	int32x4_t y = vaddq_s32(k2, vqdmulhq_s32(k3, t));
	y = vaddq_s32(k1, vqdmulhq_s32(y, t));
	y = vsraq_n_s32(b, vqdmulhq_s32(y, t), 1);
	return vqshlq_n_s32(y, 6);
}
#else
// The same arithmetic one sample at a time, for unit tests on machines without NEON. No multiply here can saturate, as
// t is never -1.
[[gnu::always_inline]] inline int32_t interpolate(int32_t x0, int32_t x1, int32_t x2, int32_t x3, int32_t t) {
	auto qdmulh = [](int32_t x, int32_t t) { return static_cast<int32_t>((int64_t{x} * t) >> 31); };
	int32_t a = x0 >> 6;
	int32_t b = x1 >> 6;
	int32_t c = x2 >> 6;
	int32_t d = x3 >> 6;
	int32_t k1 = c - a;
	int32_t k2 = 2 * a - 5 * b + 4 * c - d;
	int32_t k3 = (d - a) + 3 * (b - c);
	int32_t y = k2 + qdmulh(k3, t);
	y = k1 + qdmulh(y, t);
	y = b + (qdmulh(y, t) >> 1);
	if (y >= (1 << 25)) {
		return INT32_MAX;
	}
	if (y < -(1 << 25)) {
		return INT32_MIN;
	}
	return y << 6;
}
#endif

// The fraction of the way between source frames, Q31, from a 32.32 position
[[gnu::always_inline]] inline int32_t fraction(uint64_t position) {
	return static_cast<int32_t>(static_cast<uint32_t>(position) >> 1);
}

} // namespace catmull_rom

// Interpolates numFrames frames of kChannels interleaved samples. Output frame k sits at source position
// phase + k * increment (32.32, counting in frames from source), between frames (position >> 32) + 1 and + 2 - so
// source has to reach ((phase + (numFrames - 1) * increment) >> 32) + 3.
// All the channels share each frame's position, so they're done together, two frames at a time: each vector holds a
// pair of channels from each of the two frames.
template <size_t kChannels>
void interpolateFrames(const int32_t* source, uint64_t phase, uint64_t increment, int32_t* out, size_t numFrames) {
	static_assert(kChannels % 2 == 0, "channels are done in pairs");
	using namespace catmull_rom;
#if defined(__ARM_NEON)
	auto renderPair = [](const int32_t* xa, const int32_t* xb, int32x4_t t, int32_t* outA, int32_t* outB) {
		for (size_t c = 0; c < kChannels; c += 2) {
			int32x4_t x0 = vcombine_s32(vld1_s32(&xa[c]), vld1_s32(&xb[c]));
			int32x4_t x1 = vcombine_s32(vld1_s32(&xa[kChannels + c]), vld1_s32(&xb[kChannels + c]));
			int32x4_t x2 = vcombine_s32(vld1_s32(&xa[2 * kChannels + c]), vld1_s32(&xb[2 * kChannels + c]));
			int32x4_t x3 = vcombine_s32(vld1_s32(&xa[3 * kChannels + c]), vld1_s32(&xb[3 * kChannels + c]));
			int32x4_t y = interpolate(x0, x1, x2, x3, t);
			vst1_s32(&outA[c], vget_low_s32(y));
			if (outB != nullptr) {
				vst1_s32(&outB[c], vget_high_s32(y));
			}
		}
	};

	for (; numFrames >= 2; numFrames -= 2) {
		uint64_t nextPhase = phase + increment;
		int32x4_t t = vcombine_s32(vdup_n_s32(fraction(phase)), vdup_n_s32(fraction(nextPhase)));
		renderPair(&source[(phase >> 32) * kChannels], &source[(nextPhase >> 32) * kChannels], t, out, &out[kChannels]);
		phase = nextPhase + increment;
		out += 2 * kChannels;
	}
	if (numFrames != 0) {
		// The last one on its own
		const int32_t* x = &source[(phase >> 32) * kChannels];
		renderPair(x, x, vdupq_n_s32(fraction(phase)), out, nullptr);
	}
#else
	for (size_t k = 0; k < numFrames; k++) {
		const int32_t* x = &source[(phase >> 32) * kChannels];
		int32_t t = fraction(phase);
		for (size_t c = 0; c < kChannels; c++) {
			out[c] = interpolate(x[c], x[kChannels + c], x[2 * kChannels + c], x[3 * kChannels + c], t);
		}
		phase += increment;
		out += kChannels;
	}
#endif
}

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
		while (!out.empty()) {
			// Output frame k sits at source position phase_ + k * increment_, counting from history_[0], so this chunk
			// moves the history on by the whole part of the last one's position
			size_t numFrames = std::min(out.size(), kMaxChunkFrames);
			size_t numNew;
			while ((numNew = static_cast<size_t>((phase_ + (numFrames - 1) * increment_) >> 32)) > kMaxChunkSource) {
				numFrames /= 2;
			}

			// The history, then the source frames this chunk needs, so each output just indexes in
			std::array<Frame, kHistory + kMaxChunkSource> frames;
			std::copy(history_.begin(), history_.end(), frames.begin());
			size_t got = source.pop({&frames[kHistory], numNew});
			if (got < numNew) {
				ranDry = true;
				std::fill(&frames[kHistory + got], &frames[kHistory + numNew], Frame{0, 0});
			}

			interpolateFrames<kChannels>(reinterpret_cast<const int32_t*>(frames.data()), phase_, increment_,
			                             reinterpret_cast<int32_t*>(out.data()), numFrames);

			std::copy_n(&frames[numNew], kHistory, history_.begin());
			phase_ += numFrames * increment_ - (uint64_t{numNew} << 32);
			out = out.subspan(numFrames);
		}
		return !ranDry;
	}

private:
	static constexpr size_t kChannels = sizeof(Frame) / sizeof(int32_t);
	static constexpr size_t kHistory = 4;
	// Rendering goes in chunks of up to this many frames, through a buffer of the source frames they need on the stack.
	// That's room for an increment of up to 2, beyond which chunks just get smaller.
	static constexpr size_t kMaxChunkFrames = 64;
	static constexpr size_t kMaxChunkSource = 2 * kMaxChunkFrames;

	// The output lies between history_[1] and history_[2], phase_ (32.32) of the way
	std::array<Frame, kHistory> history_{};
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

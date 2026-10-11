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
template <size_t kCapacity>
class FrameRing {
	static_assert((kCapacity & (kCapacity - 1)) == 0, "capacity must be a power of two");

public:
	size_t size() const {
		return writeIndex_.load(std::memory_order_acquire) - readIndex_.load(std::memory_order_acquire);
	}
	size_t space() const { return kCapacity - size(); }

	// Producer side. Returns how many frames fitted - any more are dropped.
	size_t push(std::span<const Frame> frames) {
		size_t write = writeIndex_.load(std::memory_order_relaxed);
		size_t count = std::min(frames.size(), kCapacity - (write - readIndex_.load(std::memory_order_acquire)));
		for (size_t i = 0; i < count; i++) {
			buffer_[(write + i) & (kCapacity - 1)] = frames[i];
		}
		writeIndex_.store(write + count, std::memory_order_release);
		return count;
	}

	// Consumer side. Returns how many frames were available, up to dest.size().
	size_t pop(std::span<Frame> dest) {
		size_t read = readIndex_.load(std::memory_order_relaxed);
		size_t count = std::min(dest.size(), writeIndex_.load(std::memory_order_acquire) - read);
		for (size_t i = 0; i < count; i++) {
			dest[i] = buffer_[(read + i) & (kCapacity - 1)];
		}
		readIndex_.store(read + count, std::memory_order_release);
		return count;
	}

	// Consumer side
	void discard(size_t count) {
		size_t read = readIndex_.load(std::memory_order_relaxed);
		count = std::min(count, writeIndex_.load(std::memory_order_acquire) - read);
		readIndex_.store(read + count, std::memory_order_release);
	}

	// Consumer side. Only safe to call while the producer isn't pushing.
	void clear() { readIndex_.store(writeIndex_.load(std::memory_order_acquire), std::memory_order_release); }

private:
	std::array<Frame, kCapacity> buffer_{};
	std::atomic<size_t> writeIndex_{0};
	std::atomic<size_t> readIndex_{0};
};

// Decides how many frames go in each packet to the computer, so that on average they go at exactly the rate the
// codec's clock is producing them. Each packet carries the frames the codec has played since the last one -
// normally 44 or 45 - with any fraction carried over to the next. Packet sizes are kept within one frame of the
// nominal 44.1, as Windows requires (FMT-2 2.3.1.1).
class PacketSizer {
public:
	static constexpr int32_t kMinFrames = 44;
	static constexpr int32_t kMaxFrames = 45;
	// Normally only a fraction of a frame is carried over, plus a frame or so of jitter in when packets are taken
	static constexpr int32_t kMaxCarry = 2;

	void reset() { owed_ = 0; }

	// codecFramesElapsed: frames the codec has played since the previous call
	int32_t nextPacketFrames(int32_t codecFramesElapsed) {
		owed_ += codecFramesElapsed;
		int32_t frames = std::clamp(owed_, kMinFrames, kMaxFrames);
		owed_ -= frames;
		// If the computer skipped some frames, or we were held up, there's more owed than can be caught up on within
		// the limits without sending maximum-size packets for ages. Forget it - any frames that build up in the ring
		// as a result get skipped by framesToSkipForLatency().
		owed_ = std::clamp(owed_, -kMaxCarry, kMaxCarry);
		return frames;
	}

private:
	int32_t owed_ = 0;
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

} // namespace deluge::io::usb::audio

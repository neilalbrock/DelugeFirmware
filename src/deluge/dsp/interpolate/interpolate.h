#pragma once
#include "definitions_cxx.hpp"
#include "deluge/dsp/stereo_sample.h"
#include <array>
#include <cstdint>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

extern const int16_t windowedSincKernel[][17][16];

namespace deluge::dsp {
struct [[gnu::hot]] Interpolator {

	Interpolator() = default;
	// Defined in interpolate_inline.hpp, which callers must include
	[[gnu::always_inline]] inline StereoSample interpolate(size_t channels, int32_t whichKernel, uint32_t oscPos);
	[[gnu::always_inline]] inline StereoSample interpolateLinear(size_t channels, uint32_t oscPos);

	[[gnu::always_inline]] constexpr void pushL(int16_t value) {
#pragma unroll
		for (int32_t i = kInterpolationMaxNumSamples - 1; i >= 1; i--) {
			buffer_l[i] = buffer_l[i - 1];
		}
		buffer_l[0] = value;
	}

	[[gnu::always_inline]] constexpr void pushR(int16_t value) {
#pragma unroll
		for (int32_t i = kInterpolationMaxNumSamples - 1; i >= 1; i--) {
			buffer_r[i] = buffer_r[i - 1];
		}
		buffer_r[0] = value;
	}

	/// Move the history along by num_samples (0 to kInterpolationMaxNumSamples), for the right channel too only if
	/// there is one. The caller then fills [0, num_samples) with the new samples - until it does, those are 0.
	[[gnu::always_inline]] void jumpForward(size_t num_samples, size_t channels) {
		shiftAlong(buffer_l, num_samples);
		if (channels == 2) {
			shiftAlong(buffer_r, num_samples);
		}
	}

	// These are the state buffers (quadword alignment for NEON). Value-initialised: interpolate() reads the full
	// kInterpolationMaxNumSamples-tap window via SIMD, so any tap not yet pushed must be a defined 0 rather than
	// whatever the allocation left behind (the live-input pitch-shifter feeds these incrementally on warm-up).
	alignas(sizeof(int32_t) * 4) std::array<int16_t, kInterpolationMaxNumSamples> buffer_l{};
	alignas(sizeof(int32_t) * 4) std::array<int16_t, kInterpolationMaxNumSamples> buffer_r{};

private:
	using History = std::array<int16_t, kInterpolationMaxNumSamples>;

	/// history[i] = old history[i - num_samples] for i >= num_samples; [0, num_samples) become 0. The shift amount is
	/// only known at run time, so an element loop compiles to one halfword at a time. A byte table lookup does the
	/// whole 32-byte history in a few NEON instructions instead.
	[[gnu::always_inline]] static void shiftAlong(History& history, size_t num_samples) {
#if defined(__ARM_NEON)
		static constexpr uint8_t kByteIndices[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
		                                             16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
		auto* bytes = reinterpret_cast<uint8_t*>(history.data());
		uint8x16_t low = vld1q_u8(bytes);
		uint8x16_t high = vld1q_u8(bytes + 16);
		uint8x8x4_t table = {{vget_low_u8(low), vget_high_u8(low), vget_low_u8(high), vget_high_u8(high)}};
		// Source byte for each destination byte. Those before the start wrap round to 224 or more, which vtbl reads
		// as out of range and gives 0 for
		uint8x16_t shift = vdupq_n_u8(num_samples * sizeof(int16_t));
		uint8x16_t from_low = vsubq_u8(vld1q_u8(&kByteIndices[0]), shift);
		uint8x16_t from_high = vsubq_u8(vld1q_u8(&kByteIndices[16]), shift);
		vst1q_u8(bytes, vcombine_u8(vtbl4_u8(table, vget_low_u8(from_low)), vtbl4_u8(table, vget_high_u8(from_low))));
		vst1q_u8(bytes + 16,
		         vcombine_u8(vtbl4_u8(table, vget_low_u8(from_high)), vtbl4_u8(table, vget_high_u8(from_high))));
#else
		for (int32_t i = kInterpolationMaxNumSamples - 1; i >= 0; i--) {
			history[i] = (i >= (int32_t)num_samples) ? history[i - num_samples] : 0;
		}
#endif
	}
};
} // namespace deluge::dsp

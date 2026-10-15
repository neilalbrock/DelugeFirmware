/*
 * Tests for Interpolator::jumpForward() (dsp/interpolate/interpolate.h).
 *
 * jumpForward moves the sample history along by however many input samples the resampler has passed, and the caller
 * then writes the new ones into [0, n). It must leave the rest of the history exactly as the element-by-element
 * shift did, or the sinc output changes.
 */

#include "CppUTest/TestHarness.h"
#include "dsp/interpolate/interpolate.h"
#include <array>
#include <cstdint>
#include <random>

using deluge::dsp::Interpolator;

namespace {
using History = std::array<int16_t, kInterpolationMaxNumSamples>;

// What jumpForward used to do
void referenceShift(History& history, size_t n) {
	for (int32_t i = kInterpolationMaxNumSamples - 1; i >= (int32_t)n; i--) {
		history[i] = history[i - n];
	}
}

void fillRandom(History& history, std::mt19937& rng) {
	std::uniform_int_distribution<int32_t> dist(INT16_MIN, INT16_MAX);
	for (auto& v : history) {
		v = (int16_t)dist(rng);
	}
}
} // namespace

TEST_GROUP(InterpolatorJumpForward){};

TEST(InterpolatorJumpForward, matchesElementShiftForEveryAmount) {
	std::mt19937 rng(1234);
	for (size_t n = 0; n <= kInterpolationMaxNumSamples; n++) {
		for (int trial = 0; trial < 50; trial++) {
			Interpolator interpolator;
			fillRandom(interpolator.buffer_l, rng);
			fillRandom(interpolator.buffer_r, rng);
			History expectedL = interpolator.buffer_l;
			History expectedR = interpolator.buffer_r;
			referenceShift(expectedL, n);
			referenceShift(expectedR, n);

			interpolator.jumpForward(n, 2);

			for (size_t i = n; i < kInterpolationMaxNumSamples; i++) {
				CHECK_EQUAL(expectedL[i], interpolator.buffer_l[i]);
				CHECK_EQUAL(expectedR[i], interpolator.buffer_r[i]);
			}
		}
	}
}

TEST(InterpolatorJumpForward, monoLeavesRightChannelAlone) {
	std::mt19937 rng(99);
	Interpolator interpolator;
	fillRandom(interpolator.buffer_l, rng);
	fillRandom(interpolator.buffer_r, rng);
	History before = interpolator.buffer_r;

	interpolator.jumpForward(3, 1);

	for (size_t i = 0; i < kInterpolationMaxNumSamples; i++) {
		CHECK_EQUAL(before[i], interpolator.buffer_r[i]);
	}
}

TEST(InterpolatorJumpForward, matchesOverAStreamOfSamples) {
	// Push a stream through both versions as the resampler does, writing the new samples after each jump
	std::mt19937 rng(7);
	std::uniform_int_distribution<int32_t> jump(1, kInterpolationMaxNumSamples);
	std::uniform_int_distribution<int32_t> sample(INT16_MIN, INT16_MAX);
	Interpolator interpolator;
	History expected{};
	for (int step = 0; step < 2000; step++) {
		size_t n = jump(rng);
		referenceShift(expected, n);
		interpolator.jumpForward(n, 1);
		for (int32_t i = n - 1; i >= 0; i--) {
			int16_t v = (int16_t)sample(rng);
			expected[i] = v;
			interpolator.buffer_l[i] = v;
		}
		for (size_t i = 0; i < kInterpolationMaxNumSamples; i++) {
			CHECK_EQUAL(expected[i], interpolator.buffer_l[i]);
		}
	}
}

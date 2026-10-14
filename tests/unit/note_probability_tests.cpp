/*
 * Tests for sanitizeProbability() (model/note/note.h).
 *
 * Playback uses a note's probability (with the "follow previous" flag masked off) to index arrays sized
 * kNumProbabilityValues, so anything read from a file outside the valid range must become 100%.
 */

#include "CppUTest/TestHarness.h"
#include "definitions_cxx.hpp"
#include "model/note/note.h"

TEST_GROUP(NoteProbability){};

TEST(NoteProbability, validValuesAreKept) {
	for (int32_t p = 1; p <= kNumProbabilityValues; p++) {
		CHECK_EQUAL(p, sanitizeProbability(p));
	}
}

TEST(NoteProbability, validFollowValuesAreKept) {
	for (int32_t p = 1; p < kNumProbabilityValues; p++) {
		CHECK_EQUAL(p | 128, sanitizeProbability(p | 128));
	}
}

TEST(NoteProbability, zeroBecomes100Percent) {
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(0));
}

TEST(NoteProbability, followWithZeroBecomes100Percent) {
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(128));
}

TEST(NoteProbability, followWith100PercentBecomes100Percent) {
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(kNumProbabilityValues | 128));
}

TEST(NoteProbability, outOfRangeBecomes100Percent) {
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(kNumProbabilityValues + 1));
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(127));
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(255));
	CHECK_EQUAL(kNumProbabilityValues, sanitizeProbability(-1));
}

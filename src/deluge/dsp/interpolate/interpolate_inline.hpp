#pragma once
// The sinc and linear interpolation themselves. They run once per output sample of every resampled voice, so they're
// defined here for inlining into their callers (the sample reader and the live pitch shifter) rather than as calls
// into the -marm, non-LTO dsp library. Kept apart from interpolate.h so that doesn't need argon.
#include "dsp/interpolate/interpolate.h"
#include <argon.hpp>
#include <cstdint>
#include <limits>

namespace deluge::dsp {
inline StereoSample Interpolator::interpolate(size_t channels, int32_t whichKernel, uint32_t oscPos) {
	constexpr size_t numBitsInTableSize = 8;
	constexpr size_t rshiftAmount = ((24 + kInterpolationMaxNumSamplesMagnitude) - 16 - numBitsInTableSize + 1);

	StereoSample output;

	uint32_t rshifted;
	if constexpr (rshiftAmount >= 0) {
		rshifted = oscPos >> rshiftAmount;
	}
	else {
		rshifted = oscPos << (-rshiftAmount);
	}

	int16_t strength2 = rshifted & std::numeric_limits<int16_t>::max();

	int32_t progressSmall = oscPos >> (24 + kInterpolationMaxNumSamplesMagnitude - numBitsInTableSize);

	std::array<Argon<int16_t>, kInterpolationMaxNumSamples / 8> kernelVector;

	for (size_t i = 0; i < kernelVector.size(); ++i) {
		auto value1 = Argon<int16_t>::Load(&windowedSincKernel[whichKernel][progressSmall][i * 8]); // 8 lanes
		auto value2 = Argon<int16_t>::Load(&windowedSincKernel[whichKernel][progressSmall + 1][i * 8]);

		// standard linear a + (b - a) * fractional
		kernelVector[i] = value1.MultiplyAddFixedQMax((value2 - value1), strength2);
	}

	Argon<int32_t> multiplied = 0;
	for (size_t i = 0; i < kernelVector.size(); ++i) {
		size_t idx = i * decltype(kernelVector)::value_type::lanes;
		// TODO: auto [low, high] = ArgonHalf<int16_t>::LoadMulti<2>(&buffer_l[idx]);
		auto low = ArgonHalf<int16_t>::Load(&buffer_l[idx]);
		auto high = ArgonHalf<int16_t>::Load(&buffer_l[idx + ArgonHalf<int16_t>::lanes]);

		multiplied = multiplied
		                 .MultiplyAddLong(kernelVector[i].GetLow(), low)    // low half
		                 .MultiplyAddLong(kernelVector[i].GetHigh(), high); // high half
	}

	output.l = multiplied.ReduceAdd();

	if (channels == 2) {

		Argon<int32_t> multiplied = 0;
		for (size_t i = 0; i < kernelVector.size(); ++i) {
			size_t idx = i * decltype(kernelVector)::value_type::lanes;
			// TODO: auto [low, high] = ArgonHalf<int16_t>::LoadMulti<2>(&buffer_l[idx]);
			auto low = ArgonHalf<int16_t>::Load(&buffer_r[idx]);
			auto high = ArgonHalf<int16_t>::Load(&buffer_r[idx + ArgonHalf<int16_t>::lanes]);

			multiplied = multiplied
			                 .MultiplyAddLong(kernelVector[i].GetLow(), low)    // low half
			                 .MultiplyAddLong(kernelVector[i].GetHigh(), high); // high half
		}

		output.r = multiplied.ReduceAdd();
	}
	return output;
}

inline StereoSample Interpolator::interpolateLinear(size_t channels, uint32_t phase) {
	StereoSample output;
	int16_t strength2 = phase >> 9;
	int16_t strength1 = std::numeric_limits<int16_t>::max() - strength2; // inverse

	output.l = (buffer_l[1] * strength1) + (buffer_l[0] * strength2);
	if (channels == 2) {
		output.r = (buffer_r[1] * strength1) + (buffer_r[0] * strength2);
	}
	return output;
}
} // namespace deluge::dsp

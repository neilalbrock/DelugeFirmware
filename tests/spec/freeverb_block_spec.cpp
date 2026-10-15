#include "dsp/reverb/freeverb/freeverb.hpp"

#include "cppspec.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

using deluge::dsp::reverb::Freeverb;

namespace {

/// Freeverb::process() runs each comb and allpass over the whole block. This is what it did before: every filter for
/// each sample in turn, via ProcessOne(), after the same input high-pass.
struct PerSampleReference {
	int32_t reverb_send_post_lpf = 0;

	void process(Freeverb& model, std::vector<int32_t> input, std::vector<StereoSample>& output) {
		for (int32_t& reverb_sample : input) {
			int32_t distance_to_go = reverb_sample - reverb_send_post_lpf;
			reverb_send_post_lpf += distance_to_go >> 11;
			reverb_sample -= reverb_send_post_lpf;
		}
		for (size_t frame = 0; frame < input.size(); frame++) {
			model.ProcessOne(input[frame], output[frame]);
		}
	}
};

std::unique_ptr<Freeverb> makeFreeverb(float room_size, float damping, float width) {
	auto model = std::make_unique<Freeverb>();
	model->setRoomSize(room_size);
	model->setDamping(damping);
	model->setWidth(width);
	model->setPanLevels(1 << 30, (1 << 30) - 12345);
	return model;
}

/// Runs the same random signal, in blocks of varying length, through process() and the per-sample reference and
/// returns how many output samples differed.
size_t countMismatches(float room_size, float damping, float width, uint32_t seed) {
	auto block_model = makeFreeverb(room_size, damping, width);
	auto reference_model = makeFreeverb(room_size, damping, width);
	PerSampleReference reference;

	std::mt19937 rng(seed);
	std::uniform_int_distribution<int32_t> level(-(1 << 28), 1 << 28);
	// Lengths that don't divide the delay lines, so runs split at every wrap position, plus whole and single frames
	constexpr std::array<size_t, 6> kLengths = {128, 37, 1, 64, 127, 128};

	size_t mismatches = 0;
	for (size_t block = 0; block < 3000; block++) {
		size_t length = kLengths[block % kLengths.size()];
		std::vector<int32_t> input(length);
		// Bursts of noise with silences between, so the tails get exercised too
		bool silent = (block / 200) % 2 == 1;
		for (int32_t& sample : input) {
			sample = silent ? 0 : level(rng);
		}

		std::vector<StereoSample> block_output(length, StereoSample{.l = 7, .r = -7});
		std::vector<StereoSample> reference_output = block_output;

		std::vector<int32_t> block_input = input;
		block_model->process(block_input, block_output);
		reference.process(*reference_model, input, reference_output);

		for (size_t i = 0; i < length; i++) {
			if (block_output[i].l != reference_output[i].l || block_output[i].r != reference_output[i].r) {
				mismatches++;
			}
		}
	}
	return mismatches;
}

} // namespace

// clang-format off
describe freeverb_block("Freeverb block processing", $ {
	it("matches per-sample processing exactly at the default settings", _ {
		expect(countMismatches(0.5f, 0.5f, 1.0f, 1)).to_equal(0u);
	});

	it("matches per-sample processing exactly with a big damped room and narrow width", _ {
		expect(countMismatches(0.95f, 0.8f, 0.3f, 2)).to_equal(0u);
	});

	it("matches per-sample processing exactly with a small bright room", _ {
		expect(countMismatches(0.1f, 0.0f, 0.0f, 3)).to_equal(0u);
	});
});
// clang-format on

CPPSPEC_SPEC(freeverb_block)

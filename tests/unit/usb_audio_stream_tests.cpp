#include "CppUTest/TestHarness.h"
#include "io/usb/usb_audio_stream.h"
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <type_traits>
#include <vector>

using namespace deluge::io::usb::audio;

TEST_GROUP(UsbAudioFrameRingTests){};

TEST(UsbAudioFrameRingTests, pushThenPopInOrder) {
	FrameRing<8> ring;
	Frame in[3] = {{1, -1}, {2, -2}, {3, -3}};
	CHECK_EQUAL(3, ring.push(in));
	CHECK_EQUAL(3, ring.size());

	Frame out[3]{};
	CHECK_EQUAL(3, ring.pop(out));
	for (int i = 0; i < 3; i++) {
		CHECK_EQUAL(in[i].l, out[i].l);
		CHECK_EQUAL(in[i].r, out[i].r);
	}
	CHECK_EQUAL(0, ring.size());
}

TEST(UsbAudioFrameRingTests, dropsWhatDoesNotFit) {
	FrameRing<4> ring;
	Frame in[6] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}};
	CHECK_EQUAL(4, ring.push(in));
	CHECK_EQUAL(0, ring.space());

	Frame out[6]{};
	CHECK_EQUAL(4, ring.pop(out));
	CHECK_EQUAL(4, out[3].l);
}

TEST(UsbAudioFrameRingTests, wrapsAround) {
	FrameRing<4> ring;
	Frame out[4]{};
	for (int32_t i = 0; i < 20; i++) {
		Frame in[3] = {{i, 0}, {i + 100, 0}, {i + 200, 0}};
		CHECK_EQUAL(3, ring.push(in));
		CHECK_EQUAL(3, ring.pop(out));
		CHECK_EQUAL(i + 200, out[2].l);
	}
}

TEST(UsbAudioFrameRingTests, popsOnlyWhatIsThere) {
	FrameRing<8> ring;
	Frame in[2] = {{1, 0}, {2, 0}};
	ring.push(in);
	Frame out[5]{};
	CHECK_EQUAL(2, ring.pop(out));
}

TEST(UsbAudioFrameRingTests, discardSkipsOldestFrames) {
	FrameRing<8> ring;
	Frame in[4] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
	ring.push(in);
	ring.discard(3);
	Frame out[1]{};
	CHECK_EQUAL(1, ring.pop(out));
	CHECK_EQUAL(4, out[0].l);

	ring.discard(10); // More than there is
	CHECK_EQUAL(0, ring.size());
}

TEST_GROUP(UsbAudioPacketSizerTests){};

namespace {
// Frames due by time t ms, at 44.1kHz
int32_t codecFramesAt(int64_t ms) {
	return static_cast<int32_t>(ms * 44100 / 1000);
}
} // namespace

TEST(UsbAudioPacketSizerTests, sendsExactlyTheNominalRate) {
	PacketSizer sizer;
	int64_t sent = 0;
	for (int64_t ms = 1; ms <= 10000; ms++) {
		int32_t frames = sizer.nextPacketFrames();
		CHECK(frames == 44 || frames == 45);
		CHECK(frames <= PacketSizer::kMaxFrames);
		sent += frames;
		// Never a frame adrift of 44.1 per ms, and exact every 10 packets
		CHECK(std::abs(sent - codecFramesAt(ms)) <= 1);
		if (ms % 10 == 0) {
			CHECK_EQUAL(codecFramesAt(ms), sent);
		}
	}
}

TEST_GROUP(UsbAudioPackingTests){};

TEST(UsbAudioPackingTests, packsTopTwentyFourBitsLittleEndian) {
	Frame frames[2] = {{0x12345678, static_cast<int32_t>(0xFEDCBA98)}, {0x7FFFFF00, static_cast<int32_t>(0x80000000)}};
	uint8_t out[12]{};
	CHECK_EQUAL(12, packFrames24(frames, out));

	uint8_t expected[12] = {0x56, 0x34, 0x12, 0xBA, 0xDC, 0xFE, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x80};
	MEMCMP_EQUAL(expected, out, sizeof(expected));
}

TEST(UsbAudioPackingTests, maxPacketFitsTheEndpoint) {
	std::vector<Frame> frames(PacketSizer::kMaxFrames);
	std::vector<uint8_t> out(USB_AUDIO_MAX_PACKET_SIZE);
	CHECK(packFrames24(frames, out.data()) <= USB_AUDIO_MAX_PACKET_SIZE);
}

TEST_GROUP(UsbAudioLatencyTests){};

TEST(UsbAudioLatencyTests, onlySkipsWhenWellOverTarget) {
	CHECK_EQUAL(0, (framesToSkipForLatency<200, 64>(150)));
	CHECK_EQUAL(0, (framesToSkipForLatency<200, 64>(264)));
	CHECK_EQUAL(100, (framesToSkipForLatency<200, 64>(300)));
}

TEST_GROUP(UsbAudioUnpackingTests){};

TEST(UsbAudioUnpackingTests, roundTripsWithPacking) {
	Frame frames[3] = {
	    {0x12345600, static_cast<int32_t>(0xFEDCBA00)}, {0x7FFFFF00, static_cast<int32_t>(0x80000000)}, {0, -256}};
	uint8_t bytes[18]{};
	packFrames24(frames, bytes);

	Frame out[3]{};
	CHECK_EQUAL(3, unpackFrames24(bytes, sizeof(bytes), out, 3));
	for (int i = 0; i < 3; i++) {
		CHECK_EQUAL(frames[i].l, out[i].l);
		CHECK_EQUAL(frames[i].r, out[i].r);
	}
}

TEST(UsbAudioUnpackingTests, ignoresPartialFramesAndRespectsMax) {
	uint8_t bytes[20]{};
	Frame out[3]{};
	CHECK_EQUAL(3, unpackFrames24(bytes, sizeof(bytes), out, 3)); // 20 bytes is 3 whole frames
	CHECK_EQUAL(2, unpackFrames24(bytes, sizeof(bytes), out, 2));
}

TEST_GROUP(UsbAudioResamplerTests){};

namespace {
template <size_t N>
void pushRamp(FrameRing<N>& ring, int32_t first, int32_t count) {
	for (int32_t i = 0; i < count; i++) {
		Frame f{(first + i) << 8, -(first + i) << 8};
		ring.push({&f, 1});
	}
}
} // namespace

TEST(UsbAudioResamplerTests, unityRatioPassesSamplesThroughExactly) {
	FrameRing<64> ring;
	pushRamp(ring, 1000, 40);
	DriftResampler resampler;
	Frame out[32]{};
	CHECK(resampler.render(out, ring));
	// Output lags by the interpolator's history, after which it's the input exactly
	for (int i = 3; i < 32; i++) {
		CHECK_EQUAL((1000 + i - 3) << 8, out[i].l);
		CHECK_EQUAL(-(1000 + i - 3) << 8, out[i].r);
	}
}

TEST(UsbAudioResamplerTests, linearSignalStaysLinearBetweenSamples) {
	// Catmull-Rom reproduces straight lines exactly, so a ramp at half-sample offsets lands half way
	FrameRing<64> ring;
	pushRamp(ring, 0, 60);
	DriftResampler resampler;
	resampler.setIncrement(DriftResampler::kUnity / 2);
	Frame out[64]{};
	resampler.render(out, ring);
	for (int i = 10; i < 64; i++) {
		int32_t expectedTimesTwo = (i - 6); // Two outputs per input, after the history fills
		CHECK(std::abs(out[i].l - ((expectedTimesTwo << 8) / 2)) <= 256);
	}
}

TEST(UsbAudioResamplerTests, sineSurvivesSlightRatioChange) {
	constexpr double kFreq = 1000;
	constexpr double kAmplitude = (1 << 23) - 1;
	constexpr double kIncrement = 1.0002; // 200ppm
	FrameRing<8192> ring;
	for (int i = 0; i < 4096; i++) {
		int32_t s = static_cast<int32_t>(std::lround(kAmplitude * std::sin(2 * M_PI * kFreq * i / 44100))) << 8;
		Frame f{s, s};
		ring.push({&f, 1});
	}
	DriftResampler resampler;
	resampler.setIncrement(static_cast<uint64_t>(DriftResampler::kUnity * kIncrement));
	std::vector<Frame> out(4000);
	CHECK(resampler.render(out, ring));

	double worstError = 0;
	for (size_t i = 10; i < out.size(); i++) {
		// Output i sits at source position (i * increment) - 3, allowing for the history
		double position = i * kIncrement - 3;
		double ideal = kAmplitude * std::sin(2 * M_PI * kFreq * position / 44100);
		worstError = std::max(worstError, std::abs((out[i].l >> 8) - ideal));
	}
	CHECK(worstError / kAmplitude < 1e-4); // Better than -80dB
}

TEST(UsbAudioResamplerTests, matchesExactCatmullRom) {
	// Random full-scale input, so the curve often overshoots and has to clip, rendered in odd-sized pieces that cross
	// the resampler's internal chunks
	std::mt19937 rng(1);
	std::uniform_int_distribution<int32_t> random24(-(1 << 23), (1 << 23) - 1);
	for (double ratio : {0.5, 0.9997, 1.0, 1.0003, 1.7}) {
		std::vector<int32_t> left(3000);
		std::vector<int32_t> right(3000);
		FrameRing<4096> ring;
		for (size_t i = 0; i < left.size(); i++) {
			left[i] = random24(rng);
			right[i] = random24(rng);
			Frame f{left[i] << 8, right[i] << 8};
			ring.push({&f, 1});
		}

		DriftResampler resampler;
		uint64_t increment = static_cast<uint64_t>(DriftResampler::kUnity * ratio);
		resampler.setIncrement(increment);
		std::vector<Frame> out(resampler.framesAvailable(ring.size()));
		size_t done = 0;
		for (size_t piece : {1, 63, 64, 65, 200, 1, 129}) {
			piece = std::min(piece, out.size() - done);
			CHECK(resampler.render({&out[done], piece}, ring));
			done += piece;
		}
		CHECK(resampler.render({&out[done], out.size() - done}, ring));

		// Output k sits at source position k * increment, counting from the 4 frames of silent history
		auto source = [](const std::vector<int32_t>& samples, uint64_t index) {
			return index < 4 ? 0.0 : static_cast<double>(samples[index - 4]);
		};
		auto expected = [&](const std::vector<int32_t>& samples, uint64_t position) {
			uint64_t i = position >> 32;
			double t = static_cast<double>(position & 0xFFFFFFFF) / 4294967296.0;
			double a = source(samples, i);
			double b = source(samples, i + 1);
			double c = source(samples, i + 2);
			double d = source(samples, i + 3);
			double y = b + 0.5 * t * ((c - a) + t * ((2 * a - 5 * b + 4 * c - d) + t * (3 * (b - c) + d - a)));
			return std::clamp(y, -8388608.0, 8388607.75);
		};
		double worstError = 0;
		for (size_t k = 0; k < out.size(); k++) {
			uint64_t position = k * increment;
			worstError = std::max(worstError, std::abs(out[k].l / 256.0 - expected(left, position)));
			worstError = std::max(worstError, std::abs(out[k].r / 256.0 - expected(right, position)));
		}
		CHECK(worstError <= 1.0); // Within one 24-bit step
	}
}

namespace {
// Interpolates kChannels channels of random full-scale audio at random positions, numFrames at a time
template <size_t kChannels>
std::vector<int32_t> interpolateRandom(const std::vector<int32_t>& source, uint64_t phase, uint64_t increment,
                                       size_t numFrames) {
	std::vector<int32_t> out(numFrames * kChannels);
	interpolateFrames<kChannels>(source.data(), phase, increment, out.data(), numFrames);
	return out;
}

std::vector<int32_t> randomSource(size_t numSamples, uint32_t seed) {
	std::mt19937 rng(seed);
	std::uniform_int_distribution<int32_t> random24(-(1 << 23), (1 << 23) - 1);
	std::vector<int32_t> samples(numSamples);
	for (int32_t& sample : samples) {
		sample = random24(rng) << 8;
	}
	return samples;
}
} // namespace

TEST(UsbAudioResamplerTests, everyChannelCountMatchesExactCatmullRom) {
	auto check = [](auto channels) {
		constexpr size_t kChannels = decltype(channels)::value;
		constexpr size_t kNumFrames = 101; // Odd, so the last frame is done on its own
		uint64_t increment = static_cast<uint64_t>(DriftResampler::kUnity * 1.37);
		uint64_t phase = 0x89ABCDEF;
		std::vector<int32_t> source = randomSource((kNumFrames * 2 + 8) * kChannels, kChannels);
		std::vector<int32_t> out = interpolateRandom<kChannels>(source, phase, increment, kNumFrames);

		double worstError = 0;
		for (size_t k = 0; k < kNumFrames; k++) {
			uint64_t position = phase + k * increment;
			size_t i = position >> 32;
			double t = static_cast<double>(position & 0xFFFFFFFF) / 4294967296.0;
			for (size_t c = 0; c < kChannels; c++) {
				auto x = [&](size_t j) { return source[(i + j) * kChannels + c] / 256.0; };
				double a = x(0), b = x(1), cc = x(2), d = x(3);
				double y = b + 0.5 * t * ((cc - a) + t * ((2 * a - 5 * b + 4 * cc - d) + t * (3 * (b - cc) + d - a)));
				y = std::clamp(y, -8388608.0, 8388607.75);
				worstError = std::max(worstError, std::abs(out[k * kChannels + c] / 256.0 - y));
			}
		}
		CHECK(worstError <= 1.0); // Within one 24-bit step
	};
	check(std::integral_constant<size_t, 2>{});
	check(std::integral_constant<size_t, 4>{});
	check(std::integral_constant<size_t, 6>{});
	check(std::integral_constant<size_t, 8>{});
}

TEST(UsbAudioResamplerTests, channelsDontAffectEachOther) {
	// Eight channels at once give exactly what each pair does on its own
	constexpr size_t kNumFrames = 77;
	uint64_t increment = static_cast<uint64_t>(DriftResampler::kUnity * 0.9997);
	uint64_t phase = 0x7FFFFFFF;
	std::vector<int32_t> source = randomSource((kNumFrames + 8) * 8, 99);
	std::vector<int32_t> all = interpolateRandom<8>(source, phase, increment, kNumFrames);

	for (size_t pair = 0; pair < 4; pair++) {
		std::vector<int32_t> pairSource;
		for (size_t i = 0; i < source.size() / 8; i++) {
			pairSource.push_back(source[i * 8 + pair * 2]);
			pairSource.push_back(source[i * 8 + pair * 2 + 1]);
		}
		std::vector<int32_t> pairOut = interpolateRandom<2>(pairSource, phase, increment, kNumFrames);
		for (size_t k = 0; k < kNumFrames; k++) {
			CHECK_EQUAL(pairOut[k * 2], all[k * 8 + pair * 2]);
			CHECK_EQUAL(pairOut[k * 2 + 1], all[k * 8 + pair * 2 + 1]);
		}
	}
}

TEST(UsbAudioResamplerTests, reportsRunningDry) {
	FrameRing<64> ring;
	pushRamp(ring, 0, 4);
	DriftResampler resampler;
	Frame out[16]{};
	CHECK_FALSE(resampler.render(out, ring));
}

TEST(UsbAudioResamplerTests, knowsHowManyFramesItCanProduce) {
	for (double ratio : {0.5, 0.9997, 1.0, 1.0003, 1.7}) {
		DriftResampler resampler;
		resampler.setIncrement(static_cast<uint64_t>(DriftResampler::kUnity * ratio));
		FrameRing<512> ring;
		for (int block = 0; block < 20; block++) {
			pushRamp(ring, block * 37, 37); // Odd-sized blocks, so the phase lands all over the place
			size_t available = resampler.framesAvailable(ring.size());
			std::vector<Frame> out(available);
			CHECK(resampler.render(out, ring));
			CHECK_EQUAL(0, resampler.framesAvailable(ring.size()));
		}
		// One more would have needed a frame that isn't there
		Frame extra[1];
		CHECK_FALSE(resampler.render(extra, ring));
	}
}

TEST_GROUP(UsbAudioRateServoTests){};

namespace {
struct ServoRun {
	int underruns = 0;
	size_t minFill = SIZE_MAX;
	size_t maxFill = 0;
	float finalCorrectionPpm = 0;
	double meanCorrectionPpm = 0; // Over the settled half
};

// Simulates the computer sending 1ms packets on its own clock while the audio routine takes frames in blocks on the
// codec's clock, with the servo steering the resampler in between
ServoRun simulate(double hostDriftPpm, int seconds) {
	constexpr size_t kTarget = 320;
	FrameRing<1024> ring;
	DriftResampler resampler;
	RateServo servo(kTarget);
	ServoRun run;

	double hostFramesOwed = 0;
	double codecFramesOwed = 0;
	std::vector<Frame> block(256);
	Frame packet[46]{};
	bool primed = false;
	unsigned blockPattern = 0;

	for (int ms = 0; ms < seconds * 1000; ms++) {
		// One packet from the computer per ms of the computer's clock
		hostFramesOwed += 44.1 * (1 + hostDriftPpm * 1e-6);
		size_t packetFrames = static_cast<size_t>(hostFramesOwed);
		hostFramesOwed -= packetFrames;
		ring.push({packet, packetFrames});

		// The audio routine runs every 1-3ms, taking whatever the codec has used since last time
		codecFramesOwed += 44.1;
		if (++blockPattern % (1 + (ms % 3)) != 0) {
			continue;
		}
		size_t blockFrames = static_cast<size_t>(codecFramesOwed);
		codecFramesOwed -= blockFrames;

		if (!primed) {
			if (ring.size() < kTarget) {
				continue;
			}
			primed = true;
			servo.reset(ring.size(), false);
		}
		resampler.setIncrement(servo.update(ring.size(), blockFrames));
		if (!resampler.render({block.data(), blockFrames}, ring)) {
			run.underruns++;
			primed = false;
		}

		if (ms > seconds * 1000 / 2) { // After settling
			run.minFill = std::min(run.minFill, ring.size());
			run.maxFill = std::max(run.maxFill, ring.size());
		}
	}
	run.finalCorrectionPpm = servo.correctionPpm();
	return run;
}
} // namespace

TEST(UsbAudioRateServoTests, locksOntoClockDrift) {
	for (double drift : {-500.0, -200.0, 0.0, 200.0, 500.0}) {
		ServoRun run = simulate(drift, 60);
		CHECK_EQUAL(0, run.underruns);
		// Fill stays near the target, within the wobble from block sizes
		CHECK(run.minFill > 320 - 200);
		CHECK(run.maxFill < 320 + 200);
		// And the servo has learned the drift
		CHECK(std::abs(run.finalCorrectionPpm - drift) < 25);
	}
}

namespace {
// Simulates Deluge -> computer: the audio routine adds blocks on the codec's clock, which are resampled into the ring
// the ISR takes fixed-size packets from on the computer's clock - as in feedOutputToHost() and writePacketToHost()
ServoRun simulateToHost(double hostDriftPpm, int seconds) {
	constexpr size_t kTarget = 224;
	constexpr size_t kPrime = kTarget - 64;
	FrameRing<256> codecRing;
	FrameRing<512> ring;
	DriftResampler resampler;
	RateServo servo(kTarget);
	PacketSizer sizer;
	ServoRun run;

	bool primed = false;
	bool servoRunning = false;
	double codecFramesOwed = 0;
	double hostPacketsOwed = 0;
	std::vector<Frame> block(256);
	std::vector<Frame> resampled(128);
	Frame packet[PacketSizer::kMaxFrames];
	unsigned blockPattern = 0;
	double correctionSum = 0;
	int numCorrections = 0;

	for (int ms = 0; ms < seconds * 1000; ms++) {
		// The audio routine runs every 1-3ms of the codec's clock, adding what the codec has used since last time
		codecFramesOwed += 44.1;
		if (++blockPattern % (1 + (ms % 3)) == 0) {
			size_t blockFrames = static_cast<size_t>(codecFramesOwed);
			codecFramesOwed -= blockFrames;
			codecRing.push({block.data(), blockFrames});

			if (primed && !servoRunning) {
				servo.reset(ring.size(), false);
			}
			servoRunning = primed;
			size_t numResampled = 0;
			while (true) {
				size_t n = std::min({resampler.framesAvailable(codecRing.size()), ring.space(), resampled.size()});
				if (n == 0) {
					break;
				}
				resampler.render({resampled.data(), n}, codecRing);
				ring.push({resampled.data(), n});
				numResampled += n;
			}
			if (servoRunning) {
				resampler.setIncrement(servo.update(ring.size(), numResampled));
			}
			if (ms > seconds * 1000 / 2) {
				correctionSum += servo.correctionPpm();
				numCorrections++;
			}
		}

		// One packet per ms of the computer's clock
		hostPacketsOwed += 1 + hostDriftPpm * 1e-6;
		while (hostPacketsOwed >= 1) {
			hostPacketsOwed -= 1;
			size_t numFrames = sizer.nextPacketFrames();
			if (!primed && ring.size() >= kPrime) {
				primed = true;
			}
			if (primed && ring.pop({packet, numFrames}) < numFrames) {
				run.underruns++;
				primed = false;
			}
		}

		if (ms > seconds * 1000 / 2) { // After settling
			run.minFill = std::min(run.minFill, ring.size());
			run.maxFill = std::max(run.maxFill, ring.size());
		}
	}
	run.finalCorrectionPpm = servo.correctionPpm();
	run.meanCorrectionPpm = correctionSum / numCorrections;
	return run;
}
} // namespace

TEST(UsbAudioRateServoTests, locksTheOutputOntoTheComputersClock) {
	for (double drift : {-500.0, -200.0, 0.0, 200.0, 500.0}) {
		ServoRun run = simulateToHost(drift, 60);
		CHECK_EQUAL(0, run.underruns);
		CHECK(run.minFill > 0);
		CHECK(run.maxFill < 224 + 192); // Never enough to trigger skipping
		// A faster computer needs more output frames per codec frame, so fewer source frames per output. Fill is
		// measured just after bursty blocks are added, so the ratio jitters a little around that.
		CHECK(std::abs(run.meanCorrectionPpm + drift) < 25);
	}
}

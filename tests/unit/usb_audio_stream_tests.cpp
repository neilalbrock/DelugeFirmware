#include "CppUTest/TestHarness.h"
#include "io/usb/usb_audio_stream.h"
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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
// Frames the codec has played by time t ms, at 44.1kHz
int32_t codecFramesAt(int64_t ms) {
	return static_cast<int32_t>(ms * 44100 / 1000);
}
} // namespace

TEST(UsbAudioPacketSizerTests, tracksTheCodecExactly) {
	PacketSizer sizer;
	int64_t sent = 0;
	for (int64_t ms = 1; ms <= 10000; ms++) {
		int32_t frames = sizer.nextPacketFrames(codecFramesAt(ms) - codecFramesAt(ms - 1));
		CHECK(frames == 44 || frames == 45);
		sent += frames;
		// Never more than a frame adrift of the codec
		CHECK(std::abs(sent - codecFramesAt(ms)) <= 1);
	}
}

TEST(UsbAudioPacketSizerTests, followsAFastOrSlowCodecClock) {
	for (int64_t ppm : {-200, 200}) {
		PacketSizer sizer;
		int64_t sent = 0;
		int64_t played = 0;
		for (int64_t ms = 1; ms <= 20000; ms++) {
			int64_t nowPlayed = ms * 44100 * (1000000 + ppm) / 1000000000;
			int32_t frames = sizer.nextPacketFrames(static_cast<int32_t>(nowPlayed - played));
			played = nowPlayed;
			CHECK(frames == 44 || frames == 45);
			sent += frames;
		}
		CHECK(std::abs(sent - played) <= 1);
	}
}

TEST(UsbAudioPacketSizerTests, absorbsJitterInWhenPacketsAreTaken) {
	// The ISR doesn't run exactly on the 1ms boundary, so the frames counted between packets wobble
	PacketSizer sizer;
	int64_t sent = 0;
	int64_t played = 0;
	for (int64_t ms = 1; ms <= 10000; ms++) {
		int64_t jitter = (ms % 3) - 1; // -1, 0 or +1 frame
		int64_t nowPlayed = codecFramesAt(ms) + jitter;
		int32_t frames = sizer.nextPacketFrames(static_cast<int32_t>(nowPlayed - played));
		played = nowPlayed;
		CHECK(frames == 44 || frames == 45);
		sent += frames;
	}
	CHECK(std::abs(sent - played) <= 2);
}

TEST(UsbAudioPacketSizerTests, dropsBacklogAfterAMissedPacket) {
	PacketSizer sizer;
	sizer.nextPacketFrames(44);
	CHECK_EQUAL(45, sizer.nextPacketFrames(88 * 3)); // Held up for a few ms
	// Back to normal straight away, rather than sending 45s for ages to catch up
	int32_t fortyFives = 0;
	for (int i = 0; i < 20; i++) {
		fortyFives += (sizer.nextPacketFrames(44) == 45);
	}
	CHECK(fortyFives <= PacketSizer::kMaxCarry);
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

TEST(UsbAudioResamplerTests, reportsRunningDry) {
	FrameRing<64> ring;
	pushRamp(ring, 0, 4);
	DriftResampler resampler;
	Frame out[16]{};
	CHECK_FALSE(resampler.render(out, ring));
}

TEST_GROUP(UsbAudioRateServoTests){};

namespace {
struct ServoRun {
	int underruns = 0;
	size_t minFill = SIZE_MAX;
	size_t maxFill = 0;
	float finalCorrectionPpm = 0;
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

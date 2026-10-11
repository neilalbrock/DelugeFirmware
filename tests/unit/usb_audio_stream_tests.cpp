#include "CppUTest/TestHarness.h"
#include "io/usb/usb_audio_stream.h"
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

#include "CppUTest/TestHarness.h"
#include "io/usb/usb_audio_descriptor.h"
#include <cstring>
#include <map>
#include <set>
#include <utility>
#include <vector>

extern "C" {
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"
#include "deluge/drivers/usb/userdef/r_usb_pmidi_config.h"
extern uint8_t g_midi_configuration[];
}

namespace {

constexpr uint8_t kDescriptorTypeInterface = 0x04;
constexpr uint8_t kDescriptorTypeEndpoint = 0x05;
constexpr uint8_t kDescriptorTypeIAD = 0x0B;
constexpr uint8_t kDescriptorTypeCSInterface = 0x24;

struct Descriptor {
	size_t offset;
	const uint8_t* bytes;
	uint8_t length() const { return bytes[0]; }
	uint8_t type() const { return bytes[1]; }
};

uint16_t totalLength(const uint8_t* config) {
	return config[2] | (config[3] << 8);
}

std::vector<Descriptor> walk(const uint8_t* config) {
	std::vector<Descriptor> descriptors;
	size_t length = totalLength(config);
	for (size_t i = 0; i < length;) {
		Descriptor d{i, &config[i]};
		if (d.length() == 0) {
			FAIL("zero-length descriptor");
		}
		descriptors.push_back(d);
		i += d.length();
	}
	return descriptors;
}

// The bytes of an interface's descriptors, up to the next standard interface descriptor or the end
std::vector<uint8_t> interfaceBytes(const uint8_t* config, uint8_t interfaceNumber) {
	std::vector<uint8_t> bytes;
	bool inInterface = false;
	for (const Descriptor& d : walk(config)) {
		if (d.type() == kDescriptorTypeInterface) {
			inInterface = (d.bytes[2] == interfaceNumber);
		}
		if (inInterface) {
			bytes.insert(bytes.end(), d.bytes, d.bytes + d.length());
		}
	}
	return bytes;
}

} // namespace

TEST_GROUP(UsbAudioDescriptorTests){};

TEST(UsbAudioDescriptorTests, descriptorChainFillsConfiguration) {
	size_t end = 0;
	for (const Descriptor& d : walk(g_audio_configuration)) {
		end = d.offset + d.length();
	}
	CHECK_EQUAL(totalLength(g_audio_configuration), end);
}

TEST(UsbAudioDescriptorTests, audioControlTotalLengthMatchesItsDescriptors) {
	size_t declared = 0;
	size_t actual = 0;
	bool inAudioControl = false;
	for (const Descriptor& d : walk(g_audio_configuration)) {
		if (d.type() == kDescriptorTypeInterface) {
			inAudioControl = (d.bytes[2] == USB_AUDIO_IF_CONTROL);
		}
		else if (inAudioControl && d.type() == kDescriptorTypeCSInterface) {
			if (d.bytes[2] == 0x01) { // Header
				declared = d.bytes[6] | (d.bytes[7] << 8);
			}
			actual += d.length();
		}
	}
	CHECK_EQUAL(declared, actual);
}

TEST(UsbAudioDescriptorTests, interfacesAndEndpointCountsAgree) {
	std::map<std::pair<uint8_t, uint8_t>, int> declaredEndpoints;
	std::map<std::pair<uint8_t, uint8_t>, int> actualEndpoints;
	std::set<uint8_t> interfaces;
	std::pair<uint8_t, uint8_t> current;
	uint8_t iadCount = 0;

	for (const Descriptor& d : walk(g_audio_configuration)) {
		if (d.type() == kDescriptorTypeInterface) {
			current = {d.bytes[2], d.bytes[3]};
			declaredEndpoints[current] = d.bytes[4];
			actualEndpoints[current] = 0;
			interfaces.insert(d.bytes[2]);
		}
		else if (d.type() == kDescriptorTypeEndpoint) {
			actualEndpoints[current]++;
		}
		else if (d.type() == kDescriptorTypeIAD) {
			CHECK_EQUAL(0, d.bytes[2]); // bFirstInterface
			iadCount = d.bytes[3];
		}
	}

	CHECK(declaredEndpoints == actualEndpoints);
	CHECK_EQUAL(USB_AUDIO_NUM_INTERFACES, g_audio_configuration[4]);
	CHECK_EQUAL(USB_AUDIO_NUM_INTERFACES, interfaces.size());
	CHECK_EQUAL(USB_AUDIO_NUM_INTERFACES, iadCount);
	// Streaming interfaces must have an idle alt 0 with no endpoints
	CHECK_EQUAL(0, (declaredEndpoints[{USB_AUDIO_IF_STREAM_FROM_HOST, 0}]));
	CHECK_EQUAL(0, (declaredEndpoints[{USB_AUDIO_IF_STREAM_TO_HOST, 0}]));
}

// The driver gives the Nth endpoint descriptor the Nth row of the pipe table, so the rows have to line up
TEST(UsbAudioDescriptorTests, pipeTableRowsMatchEndpoints) {
	std::vector<Descriptor> endpoints;
	for (const Descriptor& d : walk(g_audio_configuration)) {
		if (d.type() == kDescriptorTypeEndpoint) {
			endpoints.push_back(d);
		}
	}

	size_t rows = 0;
	while (g_usb_pstd_eptbl_audio[rows * USB_EPL] != USB_PDTBLEND) {
		rows++;
	}
	CHECK_EQUAL(endpoints.size(), rows);

	for (size_t i = 0; i < rows; i++) {
		uint16_t pipe = g_usb_pstd_eptbl_audio[i * USB_EPL];
		const uint8_t* ep = endpoints[i].bytes;
		uint8_t address = ep[2];
		bool isochronous = (ep[3] & 0x03) == 0x01;
		uint16_t maxPacketSize = ep[4] | (ep[5] << 8);

		if (isochronous) {
			// Only PIPE1 and PIPE2 can be isochronous on the RZ/A1
			CHECK(pipe == USB_PIPE1 || pipe == USB_PIPE2);
			uint16_t bufferBytes = (((g_usb_pstd_eptbl_audio[i * USB_EPL + 2] >> 10) & 0x1F) + 1) * 64;
			CHECK(maxPacketSize <= bufferBytes);
			CHECK(maxPacketSize <= 1023); // Full speed limit
		}

		// Both streams run on the computer's clock: OUT adaptive, IN synchronous (bits 3:2 of bmAttributes)
		if (address == (USB_EP_OUT | USB_AUDIO_EP_FROM_HOST)) {
			CHECK_EQUAL(USB_CFG_PAUDIO_ISO_FROM_HOST, pipe);
			CHECK_EQUAL(0x09, ep[3]);
		}
		else if (address == (USB_EP_IN | USB_AUDIO_EP_TO_HOST)) {
			CHECK_EQUAL(USB_CFG_PAUDIO_ISO_TO_HOST, pipe);
			CHECK_EQUAL(0x0D, ep[3]);
		}
		else if (address == (USB_EP_OUT | USB_EP2)) {
			CHECK_EQUAL(USB_CFG_PMIDI_BULK_IN, pipe);
		}
		else if (address == (USB_EP_IN | USB_EP1)) {
			CHECK_EQUAL(USB_CFG_PMIDI_BULK_OUT, pipe);
		}
		else {
			FAIL("unexpected endpoint");
		}
	}
}

TEST(UsbAudioDescriptorTests, midiInterfaceSameAsMidiOnlyConfiguration) {
	std::vector<uint8_t> midiOnly = interfaceBytes(g_midi_configuration, 0);
	std::vector<uint8_t> composite = interfaceBytes(g_audio_configuration, USB_AUDIO_IF_MIDI);

	CHECK_EQUAL(midiOnly.size(), composite.size());
	CHECK_EQUAL(USB_AUDIO_IF_MIDI, composite[2]);
	composite[2] = 0; // The interface number is the only difference
	CHECK(midiOnly == composite);
}

TEST(UsbAudioDescriptorTests, packetSizeFitsAStereoFrameAtTheSampleRate) {
	// 44.1 frames per 1ms frame on average, so packets need room for 45
	CHECK(USB_AUDIO_MAX_PACKET_SIZE >= 45 * USB_AUDIO_BYTES_PER_FRAME);
	CHECK_EQUAL(0, USB_AUDIO_MAX_PACKET_SIZE % USB_AUDIO_BYTES_PER_FRAME);
}

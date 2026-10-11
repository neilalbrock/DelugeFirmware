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

#include "io/usb/usb_audio.h"
#include "definitions.h"
#include "dsp/stereo_sample.h"
#include "io/usb/usb_audio_descriptor.h"
#include "io/usb/usb_audio_stream.h"
#include "model/settings/runtime_feature_settings.h"
#include "util/fixedpoint.h"
#include <algorithm>
#include <atomic>
#include <cstddef>

extern "C" {
#include "RZA1/system/iodefine.h"
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "RZA1/usb/r_usb_basic/src/driver/inc/r_usb_pstd_hooks.h"
#include "RZA1/usb/r_usb_basic/src/hw/inc/r_usb_bitdefine.h"
#include "drivers/ssi/ssi.h"
#include "drivers/usb/userdef/r_usb_paudio_config.h"

// From r_usb_extern.h, which can't be included from C++
uint16_t usb_pstd_ctrl_read(uint32_t bsize, uint8_t* table);
void usb_pstd_ctrl_write(uint32_t bsize, uint8_t* table);
void usb_cstd_set_buf(usb_utr_t* ptr, uint16_t pipe);
}

namespace deluge::io::usb::audio {

static_assert(sizeof(Frame) == sizeof(StereoSample) && offsetof(StereoSample, l) == offsetof(Frame, l)
                  && offsetof(StereoSample, r) == offsetof(Frame, r),
              "frames are passed straight through from the audio engine");

namespace {

bool peripheralAudioEnabled = false; // The audio function is being presented to the computer
bool streamingFromHost = false;
std::atomic<bool> streamingToHost = false;

/*********************************************************************************************************************
 * Deluge -> computer
 *
 * The audio engine pushes each frame it sends to the codec into toHostRing. The USB ISR takes them out a packet at
 * a time, whenever the pipe has a buffer free (BRDY) - which, once both its buffers are full, is once per 1ms frame
 * as the computer collects them. Each packet carries as many frames as the codec has played since the last one, so
 * the stream runs exactly at the codec's rate, and the computer follows it (asynchronous).
 *
 * The ISR writes packets through the D0FIFO port, which nothing else uses, so it can never disturb MIDI's use of the
 * CFIFO port from task context.
 *********************************************************************************************************************/

// The audio routine renders up to SSI_TX_BUFFER_NUM_SAMPLES ahead of the codec, so the ring's fill level wobbles by
// that much. On top of that, keep some in hand so the ISR never runs dry.
constexpr size_t kToHostPrefillFrames = 96;
constexpr size_t kToHostTargetFrames = kToHostPrefillFrames + SSI_TX_BUFFER_NUM_SAMPLES;
constexpr size_t kToHostSlackFrames = 64;

FrameRing<512> toHostRing;
PacketSizer toHostSizer;
uint32_t toHostLastCodecFrame; // Where the codec had got to in its buffer, at the last packet
bool toHostPrimed;             // Whether enough has been queued up to start sending real audio

// Counters, for debugging
volatile uint32_t toHostUnderruns;
volatile uint32_t toHostDroppedFrames;
volatile uint32_t toHostFifoNotReady;

Frame toHostFrames[PacketSizer::kMaxFrames];
uint8_t toHostPacket[USB_AUDIO_MAX_PACKET_SIZE] __attribute__((aligned(4)));

// Which frame of its buffer the codec's DMA is reading
uint32_t codecFramePosition() {
	uint32_t bytes = (uint32_t)getTxBufferCurrentPlace() - (uint32_t)getTxBufferStart();
	return bytes / (NUM_MONO_OUTPUT_CHANNELS * sizeof(int32_t));
}

// ISR. Writes the next packet into whichever of the pipe's buffers is free.
// noinline: inlining this into both its callers crashes GCC 14.2 during LTO (internal compiler error)
[[gnu::noinline]] void writePacketToHost() {
	uint16_t fifoCtr = USB200.D0FIFOCTR;
	if (!(fifoCtr & USB_FRDY)) {
		toHostFifoNotReady = toHostFifoNotReady + 1;
		return;
	}

	uint32_t codecFrame = codecFramePosition();
	int32_t codecFramesElapsed = (codecFrame - toHostLastCodecFrame) & (SSI_TX_BUFFER_NUM_SAMPLES - 1);
	toHostLastCodecFrame = codecFrame;
	size_t numFrames = toHostSizer.nextPacketFrames(codecFramesElapsed);

	size_t got = 0;
	if (!toHostPrimed && toHostRing.size() >= kToHostPrefillFrames) {
		toHostPrimed = true;
	}
	if (toHostPrimed) {
		size_t skip = framesToSkipForLatency<kToHostTargetFrames, kToHostSlackFrames>(toHostRing.size());
		toHostRing.discard(skip);
		toHostDroppedFrames = toHostDroppedFrames + skip;

		got = toHostRing.pop(std::span{toHostFrames, numFrames});
		if (got < numFrames) {
			// Ran dry - start again from silence until there's enough queued up
			toHostUnderruns = toHostUnderruns + 1;
			toHostPrimed = false;
		}
	}
	std::fill(&toHostFrames[got], &toHostFrames[numFrames], Frame{0, 0});

	size_t numBytes = packFrames24(std::span{toHostFrames, numFrames}, toHostPacket);

	// MBW is 16-bit, and packets are always a whole number of 6-byte frames
	const uint16_t* halfwords = reinterpret_cast<const uint16_t*>(toHostPacket);
	for (size_t i = 0; i < numBytes / 2; i++) {
		USB200.D0FIFO.UINT16[H] = halfwords[i];
	}
	USB200.D0FIFOCTR = USB_BVAL; // Packet's shorter than the max packet size, so say it's done
}

void toHostPipeISR(uint16_t pipe, uint16_t event) {
	if (event == USB_INT_BRDY) {
		writePacketToHost();
	}
}

// Called from the PCD task (not the ISR) when the computer selects the streaming alternate setting. By then the
// driver has configured the pipe, with its PID at NAK.
void startStreamToHost() {
	constexpr uint16_t pipe = USB_CFG_PAUDIO_ISO_TO_HOST;

	// Nothing's being pushed or popped yet, so the ring can be reset safely
	toHostRing.clear();
	toHostSizer.reset();
	toHostPrimed = false;
	toHostLastCodecFrame = codecFramePosition();

	// Give the pipe the D0FIFO port, for CPU access (no DMA)
	USB200.D0FIFOSEL = USB_MBW_16 | pipe;
	while ((USB200.D0FIFOSEL & USB_CURPIPE) != pipe) {}

	// Write the first packet from here, before the ISR is involved. A transmitting pipe only gets a BRDY when a buffer
	// frees up - including, with double buffering, the other buffer once this one's written - so this sets the
	// cycle going. (The BRDY from configuring the pipe can't be relied on: the ISR clears it if it handles MIDI first.)
	USB200.BRDYSTS = (uint16_t)~(1u << pipe); // The manual says to clear BRDY status before accessing the FIFO
	writePacketToHost();

	usb_pstd_set_isr_pipe_handler(pipe, &toHostPipeISR);
	USB200.BRDYENB |= (1u << pipe);
	usb_cstd_set_buf(USB_NULL, pipe);

	streamingToHost = true;
}

// Called from the PCD task when the computer goes back to the idle alternate setting, or reconfigures. The driver has
// already unconfigured the pipe, which disables its interrupts.
void stopStreamToHost() {
	streamingToHost = false;
	usb_pstd_set_isr_pipe_handler(USB_CFG_PAUDIO_ISO_TO_HOST, nullptr);
	USB200.BRDYENB &= ~(1u << USB_CFG_PAUDIO_ISO_TO_HOST);
	USB200.D0FIFOSEL = 0;
}

/*********************************************************************************************************************
 * Computer -> Deluge
 *
 * The USB ISR reads each packet as it arrives (BRDY) into fromHostRing, through the D1FIFO port. At the start of each
 * audio routine, processInputFromHost() takes the frames the codec has captured since last time - still in the codec's
 * input buffer, ahead of anything that reads them - and replaces them with, or mixes in, the computer's audio,
 * resampled from the computer's clock to the codec's. So everything that uses the input - monitoring, recording,
 * the input oscillator, live effects - gets it without knowing.
 *********************************************************************************************************************/

// Enough in hand to ride out the audio routine taking frames in blocks, and the odd late packet
constexpr size_t kFromHostTargetFrames = 320;

FrameRing<1024> fromHostRing;
DriftResampler fromHostResampler;
RateServo fromHostServo{kFromHostTargetFrames};
bool fromHostPrimed; // Whether enough has been queued up to start using it

bool processingInput = false;
bool externalInputPluggedIn = false;
uint32_t inputProcessedUpTo; // Frame of the codec's input buffer that's been processed up to

// Counters, for debugging
volatile uint32_t fromHostUnderruns;
volatile uint32_t fromHostDroppedFrames;
volatile uint32_t fromHostFifoNotReady;

uint8_t fromHostPacket[USB_AUDIO_MAX_PACKET_SIZE] __attribute__((aligned(4)));
Frame fromHostFrames[USB_AUDIO_MAX_FRAMES_PER_PACKET];

// ISR. Reads the packet that's just arrived.
// noinline for the same reason as writePacketToHost()
[[gnu::noinline]] void readPacketFromHost() {
	uint16_t fifoCtr = USB200.D1FIFOCTR;
	if (!(fifoCtr & USB_FRDY)) {
		fromHostFifoNotReady = fromHostFifoNotReady + 1;
		return;
	}

	size_t numBytes = fifoCtr & USB_DTLN;
	if (numBytes == 0 || numBytes > sizeof(fromHostPacket)) {
		USB200.D1FIFOCTR = USB_BCLR; // Nothing to read, or bigger than the endpoint allows - discard
		return;
	}

	// MBW is 16-bit. Reading all the data releases the buffer.
	uint16_t* halfwords = reinterpret_cast<uint16_t*>(fromHostPacket);
	for (size_t i = 0; i < (numBytes + 1) / 2; i++) {
		halfwords[i] = USB200.D1FIFO.UINT16[H];
	}

	size_t numFrames = unpackFrames24(fromHostPacket, numBytes, fromHostFrames, USB_AUDIO_MAX_FRAMES_PER_PACKET);
	size_t pushed = fromHostRing.push({fromHostFrames, numFrames});
	fromHostDroppedFrames = fromHostDroppedFrames + (numFrames - pushed);
}

void fromHostPipeISR(uint16_t pipe, uint16_t event) {
	if (event == USB_INT_BRDY) {
		readPacketFromHost();
	}
}

// Called from the PCD task when the computer selects the streaming alternate setting. By then the driver has
// configured the pipe, with its PID at NAK.
void startStreamFromHost() {
	constexpr uint16_t pipe = USB_CFG_PAUDIO_ISO_FROM_HOST;

	// The ISR isn't pushing yet, and the audio routine (the only other user) can't run while this does
	fromHostRing.clear();
	fromHostResampler.reset();
	fromHostPrimed = false;

	// Give the pipe the D1FIFO port, for CPU access (no DMA)
	USB200.D1FIFOSEL = USB_MBW_16 | pipe;
	while ((USB200.D1FIFOSEL & USB_CURPIPE) != pipe) {}

	usb_pstd_set_isr_pipe_handler(pipe, &fromHostPipeISR);
	USB200.BRDYENB |= (1u << pipe);
	usb_cstd_set_buf(USB_NULL, pipe);

	streamingFromHost = true;
}

// Called from the PCD task when the computer goes back to the idle alternate setting, or reconfigures. The driver has
// already unconfigured the pipe, which disables its interrupts.
void stopStreamFromHost() {
	streamingFromHost = false;
	usb_pstd_set_isr_pipe_handler(USB_CFG_PAUDIO_ISO_FROM_HOST, nullptr);
	USB200.BRDYENB &= ~(1u << USB_CFG_PAUDIO_ISO_FROM_HOST);
	USB200.D1FIFOSEL = 0;
}

// The computer's audio, at the codec's rate. Silence if it isn't streaming, or until enough has been queued up.
void renderFromHost(std::span<Frame> out) {
	if (streamingFromHost && !fromHostPrimed && fromHostRing.size() >= kFromHostTargetFrames) {
		fromHostPrimed = true;
		fromHostServo.reset(fromHostRing.size(), false);
	}

	if (!streamingFromHost || !fromHostPrimed) {
		std::fill(out.begin(), out.end(), Frame{0, 0});
		return;
	}

	fromHostResampler.setIncrement(fromHostServo.update(fromHostRing.size(), out.size()));
	if (!fromHostResampler.render(out, fromHostRing)) {
		// Ran dry - silence until there's enough queued up again
		fromHostUnderruns = fromHostUnderruns + 1;
		fromHostPrimed = false;
	}
}

// Which frame of its input buffer the codec's DMA is writing
uint32_t codecInputFramePosition() {
	uint32_t bytes = (uint32_t)getRxBufferCurrentPlace() - (uint32_t)getRxBufferStart();
	return bytes / (NUM_MONO_INPUT_CHANNELS * sizeof(int32_t));
}

RuntimeFeatureStateUsbAudioInput inputSource() {
	return static_cast<RuntimeFeatureStateUsbAudioInput>(
	    runtimeFeatureSettings.get(RuntimeFeatureSettingType::UsbAudioInput));
}

// Data for control reads has to stay put until the transfer completes, after the handler has returned
uint8_t controlBuffer[16];

void putLE32(uint8_t* dest, uint32_t value) {
	dest[0] = value & 0xFF;
	dest[1] = (value >> 8) & 0xFF;
	dest[2] = (value >> 16) & 0xFF;
	dest[3] = (value >> 24) & 0xFF;
}

// Sends the first `size` bytes of controlBuffer, or fewer if the host asked for fewer
uint16_t replyWithControlBuffer(const usb_setup_t* req, size_t size) {
	usb_pstd_ctrl_read(std::min<size_t>(size, req->length), controlBuffer);
	return USB_TRUE;
}

// Requests to the clock source. It's the codec's fixed crystal, so there's just the one frequency, and it's always
// valid. (ADC-2 5.2.5.1)
uint16_t handleClockRequest(const usb_setup_t* req, uint16_t ctsq) {
	uint8_t request = req->type >> 8;
	uint8_t controlSelector = req->value >> 8;
	bool deviceToHost = req->type & USB_DEV_TO_HOST;

	if (deviceToHost) {
		if (ctsq != USB_CS_RDDS) {
			return USB_FALSE;
		}

		if (controlSelector == UAC2_CS_SAM_FREQ_CONTROL && request == UAC2_REQUEST_CUR) {
			putLE32(controlBuffer, USB_AUDIO_SAMPLE_RATE);
			return replyWithControlBuffer(req, 4);
		}

		if (controlSelector == UAC2_CS_SAM_FREQ_CONTROL && request == UAC2_REQUEST_RANGE) {
			// One subrange, a single discrete frequency: wNumSubRanges, then MIN, MAX, RES
			controlBuffer[0] = 1;
			controlBuffer[1] = 0;
			putLE32(&controlBuffer[2], USB_AUDIO_SAMPLE_RATE);
			putLE32(&controlBuffer[6], USB_AUDIO_SAMPLE_RATE);
			putLE32(&controlBuffer[10], 0);
			return replyWithControlBuffer(req, 14);
		}

		if (controlSelector == UAC2_CS_CLOCK_VALID_CONTROL && request == UAC2_REQUEST_CUR) {
			controlBuffer[0] = 1;
			return replyWithControlBuffer(req, 1);
		}

		return USB_FALSE;
	}

	// The frequency is read-only, but some hosts set it anyway. The only frequency on offer is 44.1kHz, so it can
	// only be that - accept the data and carry on.
	if (ctsq == USB_CS_WRDS && controlSelector == UAC2_CS_SAM_FREQ_CONTROL && request == UAC2_REQUEST_CUR
	    && req->length == 4) {
		usb_pstd_ctrl_write(4, controlBuffer);
		return USB_TRUE;
	}

	return USB_FALSE;
}

// Class requests are only answered for the AudioControl interface's clock. Everything else declares no controls, so
// it's right to STALL anything addressed to it.
uint16_t handleClassRequest(usb_setup_t* req, uint16_t ctsq) {
	if ((req->type & USB_BMREQUESTTYPERECIP) != USB_INTERFACE) {
		return USB_FALSE;
	}

	uint8_t interfaceNumber = req->index & 0xFF;
	uint8_t entityID = req->index >> 8;

	if (interfaceNumber == USB_AUDIO_IF_CONTROL && entityID == USB_AUDIO_ID_CLOCK) {
		return handleClockRequest(req, ctsq);
	}

	return USB_FALSE;
}

void handleInterfaceChanged(uint16_t interfaceNumber, uint16_t alternateSetting) {
	bool streaming = (alternateSetting == 1);

	switch (interfaceNumber) {
	case USB_AUDIO_IF_STREAM_FROM_HOST:
		if (streaming) {
			startStreamFromHost();
		}
		else if (streamingFromHost) {
			stopStreamFromHost();
		}
		break;

	case USB_AUDIO_IF_STREAM_TO_HOST:
		if (streaming) {
			startStreamToHost();
		}
		else if (streamingToHost) {
			stopStreamToHost();
		}
		break;

	default:
		break;
	}
	// Going back to alt 0 needs nothing here - the driver unconfigures the stream's pipe
}

} // namespace

bool isStreamingFromHost() {
	return streamingFromHost;
}

bool isStreamingToHost() {
	return streamingToHost.load(std::memory_order_relaxed);
}

bool isInputFromHostSelected() {
	return peripheralAudioEnabled && inputSource() != RuntimeFeatureStateUsbAudioInput::LineIn;
}

void setExternalInputPluggedIn(bool pluggedIn) {
	externalInputPluggedIn = pluggedIn;
}

void processInputFromHost() {
	if (!isInputFromHostSelected()) {
		processingInput = false;
		return;
	}

	uint32_t capturedUpTo = codecInputFramePosition();
	if (!processingInput) {
		// Just switched on. Start from here - anything before was captured as plain line in.
		processingInput = true;
		inputProcessedUpTo = capturedUpTo;
		return;
	}

	// Mixing with the internal mic would just add room noise, so only mix when something's plugged in
	bool replace = (inputSource() == RuntimeFeatureStateUsbAudioInput::Usb) || !externalInputPluggedIn;
	volatile int32_t* input = getRxBufferStart();
	size_t numFrames = (capturedUpTo - inputProcessedUpTo) & (SSI_RX_BUFFER_NUM_SAMPLES - 1);

	Frame block[SSI_TX_BUFFER_NUM_SAMPLES];
	while (numFrames > 0) {
		size_t blockFrames = std::min(numFrames, std::size(block));
		renderFromHost({block, blockFrames});

		for (size_t i = 0; i < blockFrames; i++) {
			volatile int32_t* frame = &input[inputProcessedUpTo * NUM_MONO_INPUT_CHANNELS];
			if (replace) {
				frame[0] = block[i].l;
				frame[1] = block[i].r;
			}
			else {
				frame[0] = add_saturate(frame[0], block[i].l);
				frame[1] = add_saturate(frame[1], block[i].r);
			}
			inputProcessedUpTo = (inputProcessedUpTo + 1) & (SSI_RX_BUFFER_NUM_SAMPLES - 1);
		}
		numFrames -= blockFrames;
	}
}

void feedOutputToHost(std::span<const StereoSample> frames) {
	if (!streamingToHost.load(std::memory_order_acquire)) {
		return;
	}
	size_t pushed = toHostRing.push({reinterpret_cast<const Frame*>(frames.data()), frames.size()});
	toHostDroppedFrames = toHostDroppedFrames + (frames.size() - pushed);
}

} // namespace deluge::io::usb::audio

using namespace deluge::io::usb::audio;

extern "C" void usbAudioPeripheralOpening(uint8_t enabled) {
	if (streamingToHost) {
		stopStreamToHost();
	}
	if (streamingFromHost) {
		stopStreamFromHost();
	}
	peripheralAudioEnabled = enabled;
	usb_pstd_set_class_request_handler(enabled ? &handleClassRequest : nullptr);
	usb_pstd_set_interface_changed_handler(enabled ? &handleInterfaceChanged : nullptr);
}

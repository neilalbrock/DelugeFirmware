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
#include "processing/engines/audio_engine.h"
#include <algorithm>
#include <arm_neon.h>
#include <atomic>
#include <cstddef>
#include <cstring>

extern "C" {
#include "OSLikeStuff/timers_interrupts/timers_interrupts.h"
#include "RZA1/cache/cache.h"
#include "RZA1/cpu_specific.h"
#include "RZA1/intc/devdrv_intc.h"
#include "RZA1/system/iobitmasks/dmac_iobitmask.h"
#include "RZA1/system/iodefine.h"
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "RZA1/usb/r_usb_basic/src/driver/inc/r_usb_pstd_hooks.h"
#include "RZA1/usb/r_usb_basic/src/hw/inc/r_usb_bitdefine.h"
#include "drivers/dmac/dmac.h"
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
 * DMA
 *
 * Each access to the USB FIFO ports keeps the CPU waiting for about 340ns, which over a whole packet adds up to several
 * percent of the CPU. So each stream has a DMA channel to move its packets between the FIFO port and a buffer in RAM.
 * The ISR just fills or empties the buffer, and an interrupt at the end of each DMA transfer finishes the packet off.
 *
 * The end-of-transfer handlers use memcpy, which is NEON code, so they're wrapped to save the NEON registers - as is
 * the USB ISR itself, in r_usb_usbif_api.c.
 *
 * The DMA uses the buffers' real addresses, and the CPU the uncached mirror, so the cache never needs cleaning or
 * invalidating - other than once, in case anything's left there from when the buffers were zeroed at startup. Each
 * buffer has whole cache lines to itself, so no caching of anything else can write over it.
 *********************************************************************************************************************/

constexpr size_t kDmaBufferSize = (USB_AUDIO_MAX_PACKET_SIZE + CACHE_LINE_SIZE - 1) & ~size_t{CACHE_LINE_SIZE - 1};

// The same as the USB interrupt's, so neither interrupts the other
constexpr uint8_t kDmaInterruptPriority = 9;

// 32 bits at a time, one transfer per request from the FIFO port, which holds its request line high while it's ready
// (RZ/A1L HW manual table 9.4)
constexpr uint32_t kDmaConfig = DMAC0_CHCFG_n_LVL | DMAC0_CHCFG_n_HIEN | (0b010 << 8) // AM: bus cycle mode
                                | (0b0010 << 16)                                      // DDS: 32 bits
                                | (0b0010 << 12);                                     // SDS: 32 bits

template <typename T>
T* uncached(T* address) {
	return reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(address) + UNCACHED_MIRROR_OFFSET);
}

void stopDma(uint32_t channel) {
	auto& dma = DMACn(channel);
	dma.CHCTRL_n = DMAC0_CHCTRL_n_CLREN;
	while (dma.CHSTAT_n & (DMAC0_CHSTAT_n_EN | DMAC0_CHSTAT_n_TACT)) {}
	dma.CHCTRL_n = DMAC0_CHCTRL_n_SWRST; // Clears its status, including any end of transfer not yet handled
}

void setUpDma(uint32_t channel, uint32_t config, uint32_t source, uint32_t destination, uint32_t dmars,
              void (*endHandler)(uint32_t), uint8_t* buffer) {
	invalidate_range_all_caches(reinterpret_cast<uintptr_t>(buffer),
	                            reinterpret_cast<uintptr_t>(buffer) + kDmaBufferSize);
	stopDma(channel);
	auto& dma = DMACn(channel);
	dma.CHCFG_n = config | (channel & DMAC0_CHCFG_n_SEL);
	dma.CHITVL_n = 0;
	dma.CHEXT_n = 0;
	dma.N0SA_n = source;
	dma.N0DA_n = destination;
	setDMARS(channel, dmars);
	setupAndEnableInterrupt(endHandler, DMA_INTERRUPT_0 + channel, kDmaInterruptPriority);
}

// Transfers numBytes, a multiple of 4, between the channel's buffer and FIFO port
void startDma(uint32_t channel, uint32_t numBytes) {
	asm volatile("dsb" ::: "memory"); // Anything the CPU has written to the buffer has to reach RAM first
	auto& dma = DMACn(channel);
	dma.N0TB_n = numBytes;
	dma.CHCTRL_n = DMAC0_CHCTRL_n_SWRST; // Clears the last transfer's status. Doesn't work together with SETEN.
	dma.CHCTRL_n = DMAC0_CHCTRL_n_SETEN;
}

// For the end-of-transfer interrupt: whether the channel has really finished a transfer - rather than, say, been
// stopped since - and if so, clears that
bool dmaEnded(uint32_t channel) {
	auto& dma = DMACn(channel);
	if (!(dma.CHSTAT_n & DMAC0_CHSTAT_n_END)) {
		return false;
	}
	dma.CHCTRL_n = DMAC0_CHCTRL_n_CLREND;
	return true;
}

/*********************************************************************************************************************
 * Deluge -> computer
 *
 * The stream is synchronous: it runs on the computer's clock, at 44.1 frames per 1ms USB frame, so that as far as the
 * computer is concerned both streams share one clock. The audio engine's output is on the codec's clock, so it's
 * resampled to match: feedOutputToHost() queues each block in toHostResampler, and once a batch has built up there,
 * resamples as much of it as it can straight into toHostRing, with toHostServo steering the ratio to keep toHostRing's
 * fill level steady.
 *
 * The USB ISR takes frames out of toHostRing a packet at a time, whenever the pipe has a buffer free (BRDY) - which,
 * once both its buffers are full, is once per 1ms frame as the computer collects them. The DMA writes them through the
 * D0FIFO port, which nothing else uses, so it can never disturb MIDI's use of the CFIFO port from task context.
 *********************************************************************************************************************/

// toHostRing's fill level just after each block is added. The audio routine adds blocks of up to
// SSI_TX_BUFFER_NUM_SAMPLES while the ISR takes frames steadily, so in between it dips by up to that much.
constexpr size_t kToHostTargetFrames = 224;
// The ISR starts sending real audio once this much is queued
constexpr size_t kToHostPrimeFrames = kToHostTargetFrames - SSI_TX_BUFFER_NUM_SAMPLES / 2;
// Frames only pile up beyond the dips and peaks if the computer stops collecting for a while
constexpr size_t kToHostSlackFrames = 192;

// The audio routine often outputs only a few frames at a time, and each pass of the resampler costs far more to set up
// (and steer) than per frame, so it waits for this many. Like a block of SSI_TX_BUFFER_NUM_SAMPLES, it's within the
// dips kToHostTargetFrames allows for.
constexpr size_t kToHostResampleBatchFrames = 64;

// Audio routine only. Queues the codec's output: room for a batch that's not quite complete, plus a whole block.
DriftResampler toHostResampler;
static_assert(kToHostResampleBatchFrames - 1 + SSI_TX_BUFFER_NUM_SAMPLES <= DriftResampler::kMaxQueuedFrames);
RateServo toHostServo{kToHostTargetFrames};
bool toHostServoRunning;

// Audio routine -> ISR: frames on the computer's clock
FrameRing<512> toHostRing;
PacketSizer toHostSizer;
std::atomic<bool> toHostPrimed; // Whether enough has been queued up to start sending real audio. Set by the ISR.

// Counters, for debugging
volatile uint32_t toHostUnderruns;
volatile uint32_t toHostDroppedFrames;
volatile uint32_t toHostFifoNotReady;

Frame toHostFrames[PacketSizer::kMaxFrames];
uint8_t toHostPacket[USB_AUDIO_MAX_PACKET_SIZE] __attribute__((aligned(4)));

// The DMA writes the packet's whole words, then the CPU any halfword left at the end
alignas(CACHE_LINE_SIZE) uint8_t toHostDmaBuffer[kDmaBufferSize];
constexpr uint16_t kToHostFifoSel = USB_DREQE | USB_CFG_PAUDIO_ISO_TO_HOST;
bool toHostDmaBusy;
bool toHostPacketWanted; // The pipe had a buffer free while the DMA was busy
bool toHostEndsInHalfword;
uint16_t toHostFinalHalfword;

// ISR. Starts writing the next packet into whichever of the pipe's buffers is free.
// GCC 14.2's predictive commoning pass crashes on this during LTO (internal compiler error), whether it's inlined into
// its callers or not
[[gnu::noinline, gnu::optimize("no-predictive-commoning")]] void writePacketToHost() {
	uint16_t fifoCtr = USB200.D0FIFOCTR;
	if (!(fifoCtr & USB_FRDY)) {
		toHostFifoNotReady = toHostFifoNotReady + 1;
		return;
	}

	size_t numFrames = toHostSizer.nextPacketFrames();

	size_t got = 0;
	bool primed = toHostPrimed.load(std::memory_order_relaxed);
	if (!primed && toHostRing.size() >= kToHostPrimeFrames) {
		primed = true;
	}
	if (primed) {
		size_t skip = framesToSkipForLatency<kToHostTargetFrames, kToHostSlackFrames>(toHostRing.size());
		toHostRing.discard(skip);
		toHostDroppedFrames = toHostDroppedFrames + skip;

		got = toHostRing.pop(std::span{toHostFrames, numFrames});
		if (got < numFrames) {
			// Ran dry - start again from silence until there's enough queued up
			toHostUnderruns = toHostUnderruns + 1;
			primed = false;
		}
	}
	toHostPrimed.store(primed, std::memory_order_relaxed);
	std::fill(&toHostFrames[got], &toHostFrames[numFrames], Frame{0, 0});

	size_t numBytes = packFrames24(std::span{toHostFrames, numFrames}, toHostPacket);

	// Packets are whole 6-byte frames, so one with an odd number of frames ends in a halfword
	size_t dmaBytes = numBytes & ~size_t{3};
	toHostEndsInHalfword = (numBytes % 4) != 0;
	if (toHostEndsInHalfword) {
		toHostFinalHalfword = reinterpret_cast<const uint16_t*>(toHostPacket)[numBytes / 2 - 1];
	}
	std::memcpy(uncached(toHostDmaBuffer), toHostPacket, dmaBytes);
	toHostDmaBusy = true;
	startDma(USB_AUDIO_TO_HOST_DMA_CHANNEL, dmaBytes);
}

// DMA end interrupt. Finishes off the packet.
void toHostDmaEnded(uint32_t) {
	if (!dmaEnded(USB_AUDIO_TO_HOST_DMA_CHANNEL)) {
		return;
	}
	toHostDmaBusy = false;
	if (!streamingToHost.load(std::memory_order_relaxed)) {
		return;
	}

	// While sending, the access width can be narrowed part way through a packet but not widened (RZ/A1L HW manual,
	// DnFIFOSEL.MBW), so a final halfword is written 16 bits wide, and it goes back to 32 bits once the packet's handed
	// over
	if (toHostEndsInHalfword) {
		USB200.D0FIFOSEL = kToHostFifoSel | USB_MBW_16;
		USB200.D0FIFO.UINT16[H] = toHostFinalHalfword;
	}
	USB200.D0FIFOCTR = USB_BVAL; // Packet's shorter than the max packet size, so say it's done
	if (toHostEndsInHalfword) {
		USB200.D0FIFOSEL = kToHostFifoSel | USB_MBW_32;
	}

	if (toHostPacketWanted) {
		toHostPacketWanted = false;
		writePacketToHost();
	}
}

DEFINE_FPU_PRESERVING_INTERRUPT_HANDLER(toHostDmaEndedPreservingFpu, toHostDmaEnded);

void toHostPipeISR(uint16_t pipe, uint16_t event) {
	if (event != USB_INT_BRDY) {
		return;
	}
	if (toHostDmaBusy) {
		toHostPacketWanted = true; // Once this packet's done
		return;
	}
	writePacketToHost();
}

// Called from the PCD task (not the ISR) when the computer selects the streaming alternate setting. By then the
// driver has configured the pipe, with its PID at NAK.
void startStreamToHost() {
	constexpr uint16_t pipe = USB_CFG_PAUDIO_ISO_TO_HOST;

	// Nothing's being pushed or popped yet - the ISR isn't involved, and the audio routine can't run while this does
	toHostResampler.reset();
	toHostResampler.setIncrement(DriftResampler::kUnity);
	toHostServoRunning = false;
	toHostRing.clear();
	toHostSizer.reset();
	toHostPrimed = false;
	toHostDmaBusy = false;
	toHostPacketWanted = false;

	// Give the pipe the D0FIFO port, in cycle steal mode: the DMA writes through the port itself, a word at a time
	USB200.D0FBCFG = 0;
	USB200.D0FIFOSEL = kToHostFifoSel | USB_MBW_32;
	while ((USB200.D0FIFOSEL & USB_CURPIPE) != pipe) {}
	setUpDma(USB_AUDIO_TO_HOST_DMA_CHANNEL, kDmaConfig | DMAC0_CHCFG_n_REQD | DMAC0_CHCFG_n_DAD,
	         reinterpret_cast<uint32_t>(toHostDmaBuffer), reinterpret_cast<uint32_t>(&USB200.D0FIFO.UINT32),
	         DMARS_FOR_USB0_D0FIFO, &toHostDmaEndedPreservingFpu, toHostDmaBuffer);

	// Before the first packet, as the end of its DMA transfer checks this
	streamingToHost = true;

	// Write the first packet from here, before the ISR is involved. A transmitting pipe only gets a BRDY when a buffer
	// frees up - including, with double buffering, the other buffer once this one's written - so this sets the
	// cycle going. (The BRDY from configuring the pipe can't be relied on: the ISR clears it if it handles MIDI first.)
	USB200.BRDYSTS = (uint16_t)~(1u << pipe); // The manual says to clear BRDY status before accessing the FIFO
	writePacketToHost();

	usb_pstd_set_isr_pipe_handler(pipe, &toHostPipeISR);
	USB200.BRDYENB |= (1u << pipe);
	usb_cstd_set_buf(USB_NULL, pipe);
}

// Called from the PCD task when the computer goes back to the idle alternate setting, or reconfigures. The driver has
// already unconfigured the pipe, which disables its interrupts.
void stopStreamToHost() {
	streamingToHost = false;
	usb_pstd_set_isr_pipe_handler(USB_CFG_PAUDIO_ISO_TO_HOST, nullptr);
	USB200.BRDYENB &= ~(1u << USB_CFG_PAUDIO_ISO_TO_HOST);
	stopDma(USB_AUDIO_TO_HOST_DMA_CHANNEL);
	USB200.D0FIFOSEL = 0;
}

/*********************************************************************************************************************
 * Computer -> Deluge
 *
 * Each packet is read as it arrives (BRDY) into fromHostRing - by DMA, through the D1FIFO port. Before each render,
 * processInputFromHost() makes sure the frames of the codec's input buffer that the render will read have had the
 * computer's audio put in place of what the codec captured, or mixed in, resampled from the computer's clock to the
 * codec's. So everything that uses the input - monitoring, recording, the input oscillator, live effects - gets it
 * without knowing.
 *
 * The audio routine often renders only a few frames at a time, and each pass of the resampler costs far more to set up
 * than per frame, so whenever it's needed, it also processes up to a batch more of what the codec has captured - which
 * then lasts the next several renders.
 *********************************************************************************************************************/

// Processed beyond what each render needs, when there's any processing to do
constexpr size_t kInputBatchFrames = 64;

// Enough in hand to ride out the audio routine taking frames in blocks - of up to SSI_TX_BUFFER_NUM_SAMPLES, plus a
// batch - and the odd late packet
constexpr size_t kFromHostTargetFrames = 384;

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

uint32_t fromHostPacket[(USB_AUDIO_MAX_PACKET_SIZE + 3) / 4]; // Whole words, as it's read 32 bits at a time
Frame fromHostFrames[USB_AUDIO_MAX_FRAMES_PER_PACKET];

alignas(CACHE_LINE_SIZE) uint8_t fromHostDmaBuffer[kDmaBufferSize];
bool fromHostDmaBusy;
bool fromHostPacketWaiting; // Another packet arrived while the DMA was busy
size_t fromHostPacketBytes; // Size of the packet the DMA's reading

// ISR. Starts reading the packet that's just arrived.
void readPacketFromHost() {
	uint16_t fifoCtr = USB200.D1FIFOCTR;
	if (!(fifoCtr & USB_FRDY)) {
		fromHostFifoNotReady = fromHostFifoNotReady + 1;
		return;
	}

	size_t numBytes = fifoCtr & USB_DTLN;
	if (numBytes == 0 || numBytes > USB_AUDIO_MAX_PACKET_SIZE) {
		USB200.D1FIFOCTR = USB_BCLR; // Nothing to read, or bigger than the endpoint allows - discard
		return;
	}

	// In whole words. Reading all the data releases the buffer - a packet that ends part way through a word just
	// leaves the rest of the last one unused.
	fromHostPacketBytes = numBytes;
	fromHostDmaBusy = true;
	startDma(USB_AUDIO_FROM_HOST_DMA_CHANNEL, (numBytes + 3) & ~size_t{3});
}

// DMA end interrupt. Takes the frames out of the packet.
// noinline for the same reason as writePacketToHost()
[[gnu::noinline]] void fromHostDmaEnded(uint32_t) {
	if (!dmaEnded(USB_AUDIO_FROM_HOST_DMA_CHANNEL)) {
		return;
	}
	fromHostDmaBusy = false;
	if (!streamingFromHost) {
		return;
	}

	size_t numBytes = fromHostPacketBytes;
	std::memcpy(fromHostPacket, uncached(fromHostDmaBuffer), (numBytes + 3) & ~size_t{3});
	size_t numFrames = unpackFrames24(reinterpret_cast<const uint8_t*>(fromHostPacket), numBytes, fromHostFrames,
	                                  USB_AUDIO_MAX_FRAMES_PER_PACKET);
	size_t pushed = fromHostRing.push({fromHostFrames, numFrames});
	fromHostDroppedFrames = fromHostDroppedFrames + (numFrames - pushed);

	if (fromHostPacketWaiting) {
		fromHostPacketWaiting = false;
		readPacketFromHost();
	}
}

DEFINE_FPU_PRESERVING_INTERRUPT_HANDLER(fromHostDmaEndedPreservingFpu, fromHostDmaEnded);

void fromHostPipeISR(uint16_t pipe, uint16_t event) {
	if (event != USB_INT_BRDY) {
		return;
	}
	if (fromHostDmaBusy) {
		fromHostPacketWaiting = true; // Once this packet's done
		return;
	}
	readPacketFromHost();
}

// Called from the PCD task when the computer selects the streaming alternate setting. By then the driver has
// configured the pipe, with its PID at NAK.
void startStreamFromHost() {
	constexpr uint16_t pipe = USB_CFG_PAUDIO_ISO_FROM_HOST;

	// The ISR isn't pushing yet, and the audio routine (the only other user) can't run while this does
	fromHostRing.clear();
	fromHostResampler.reset();
	fromHostPrimed = false;
	fromHostDmaBusy = false;
	fromHostPacketWaiting = false;

	// Give the pipe the D1FIFO port, in cycle steal mode: the DMA reads through the port itself, a word at a time
	USB200.D1FBCFG = 0;
	USB200.D1FIFOSEL = USB_DREQE | USB_MBW_32 | pipe;
	while ((USB200.D1FIFOSEL & USB_CURPIPE) != pipe) {}
	setUpDma(USB_AUDIO_FROM_HOST_DMA_CHANNEL, kDmaConfig | DMAC0_CHCFG_n_SAD,
	         reinterpret_cast<uint32_t>(&USB200.D1FIFO.UINT32), reinterpret_cast<uint32_t>(fromHostDmaBuffer),
	         DMARS_FOR_USB0_D1FIFO, &fromHostDmaEndedPreservingFpu, fromHostDmaBuffer);

	// Before any packets can arrive, as the end of each one's DMA transfer checks this
	streamingFromHost = true;

	usb_pstd_set_isr_pipe_handler(pipe, &fromHostPipeISR);
	USB200.BRDYENB |= (1u << pipe);
	usb_cstd_set_buf(USB_NULL, pipe);
}

// Called from the PCD task when the computer goes back to the idle alternate setting, or reconfigures. The driver has
// already unconfigured the pipe, which disables its interrupts.
void stopStreamFromHost() {
	streamingFromHost = false;
	usb_pstd_set_isr_pipe_handler(USB_CFG_PAUDIO_ISO_FROM_HOST, nullptr);
	USB200.BRDYENB &= ~(1u << USB_CFG_PAUDIO_ISO_FROM_HOST);
	stopDma(USB_AUDIO_FROM_HOST_DMA_CHANNEL);
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

// Adds source to dest, saturating, two frames at a time
void mixFrames(Frame* dest, const Frame* source, size_t numFrames) {
	auto* d = reinterpret_cast<int32_t*>(dest);
	auto* s = reinterpret_cast<const int32_t*>(source);
	for (; numFrames >= 2; numFrames -= 2, d += 4, s += 4) {
		vst1q_s32(d, vqaddq_s32(vld1q_s32(d), vld1q_s32(s)));
	}
	if (numFrames != 0) {
		vst1_s32(d, vqadd_s32(vld1_s32(d), vld1_s32(s)));
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

void processInputFromHost(size_t numFramesToRead) {
	if (!isInputFromHostSelected()) {
		processingInput = false;
		return;
	}

	// The DMA's last few writes before the position it reports can still be on their way to memory - if we replaced
	// those frames now, the codec's input would land on top afterwards. So stay a few frames back, as the audio
	// engine's own read position does (see AudioEngine::init()). Renders read from 128+ frames further back, so they
	// only reach this close when rendering a whole SSI_TX_BUFFER_NUM_SAMPLES at the shortest of those distances.
	constexpr uint32_t kDmaSettleFrames = 8;
	uint32_t capturedUpTo = (codecInputFramePosition() - kDmaSettleFrames) & (SSI_RX_BUFFER_NUM_SAMPLES - 1);
	if (!processingInput) {
		// Just switched on. Start from here - anything before was captured as plain line in.
		processingInput = true;
		inputProcessedUpTo = capturedUpTo;
		return;
	}

	// What the render will read, from AudioEngine::i2sRXBufferPos
	constexpr uint32_t kFrameMask = SSI_RX_BUFFER_NUM_SAMPLES - 1;
	uint32_t readFrom = (AudioEngine::i2sRXBufferPos - reinterpret_cast<uint32_t>(getRxBufferStart()))
	                    / (NUM_MONO_INPUT_CHANNELS * sizeof(int32_t));
	uint32_t neededUpTo = (readFrom + numFramesToRead) & kFrameMask;
	size_t numFrames = inputFramesToProcess<SSI_RX_BUFFER_NUM_SAMPLES>(inputProcessedUpTo, capturedUpTo, neededUpTo,
	                                                                   kInputBatchFrames);

	// Mixing with the internal mic would just add room noise, so only mix when something's plugged in
	bool replace = (inputSource() == RuntimeFeatureStateUsbAudioInput::Usb) || !externalInputPluggedIn;
	// Only the codec's DMA writes to the buffer as well, and only to frames ahead of these, so it needn't be volatile
	static_assert(NUM_MONO_INPUT_CHANNELS == 2, "the codec's input frames are stereo, like Frame");
	auto* input = reinterpret_cast<Frame*>(const_cast<int32_t*>(getRxBufferStart()));

	Frame block[SSI_TX_BUFFER_NUM_SAMPLES];
	while (numFrames > 0) {
		// The buffer's uncached, so it's written in runs rather than a sample at a time - each up to its end
		size_t runFrames = std::min(numFrames, static_cast<size_t>(SSI_RX_BUFFER_NUM_SAMPLES - inputProcessedUpTo));
		Frame* dest = &input[inputProcessedUpTo];
		if (replace) {
			renderFromHost({dest, runFrames}); // Straight in
		}
		else {
			runFrames = std::min(runFrames, std::size(block));
			renderFromHost({block, runFrames});
			mixFrames(dest, block, runFrames);
		}
		inputProcessedUpTo = (inputProcessedUpTo + runFrames) & (SSI_RX_BUFFER_NUM_SAMPLES - 1);
		numFrames -= runFrames;
	}
}

void feedOutputToHost(std::span<const StereoSample> frames) {
	if (!streamingToHost.load(std::memory_order_acquire)) {
		return;
	}
	size_t queued = toHostResampler.addSource({reinterpret_cast<const Frame*>(frames.data()), frames.size()});
	toHostDroppedFrames = toHostDroppedFrames + (frames.size() - queued);
	if (toHostResampler.queued() < kToHostResampleBatchFrames) {
		return;
	}

	// Only steer while the ISR is taking frames. Before that, toHostRing just fills up to where it starts.
	bool primed = toHostPrimed.load(std::memory_order_relaxed);
	if (primed && !toHostServoRunning) {
		toHostServo.reset(toHostRing.size(), false);
	}
	toHostServoRunning = primed;

	// Straight into toHostRing - in two goes if its free space wraps around the end
	size_t numResampled = 0;
	while (true) {
		std::span<Frame> space = toHostRing.writable();
		size_t numFrames = std::min(space.size(), toHostResampler.framesAvailable());
		if (numFrames == 0) {
			break;
		}
		toHostResampler.render(space.first(numFrames));
		toHostRing.commit(numFrames);
		numResampled += numFrames;
	}

	if (toHostServoRunning) {
		toHostResampler.setIncrement(toHostServo.update(toHostRing.size(), numResampled));
	}
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

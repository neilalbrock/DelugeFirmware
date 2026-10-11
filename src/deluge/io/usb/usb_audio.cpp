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
#include "io/usb/usb_audio_descriptor.h"
#include <algorithm>
#include <cstddef>

extern "C" {
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "RZA1/usb/r_usb_basic/src/driver/inc/r_usb_pstd_hooks.h"
#include "RZA1/usb/r_usb_basic/src/hw/inc/r_usb_bitdefine.h"
#include "drivers/usb/userdef/r_usb_paudio_config.h"

// From r_usb_extern.h, which can't be included from C++
uint16_t usb_pstd_ctrl_read(uint32_t bsize, uint8_t* table);
void usb_pstd_ctrl_write(uint32_t bsize, uint8_t* table);
void usb_cstd_set_buf(usb_utr_t* ptr, uint16_t pipe);
}

namespace deluge::io::usb::audio {

namespace {

bool streamingFromHost = false;
bool streamingToHost = false;

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
		streamingFromHost = streaming;
		if (streaming) {
			// Accept packets. Nothing reads them yet, so once the buffers are full they're dropped.
			usb_cstd_set_buf(USB_NULL, USB_CFG_PAUDIO_ISO_FROM_HOST);
		}
		break;

	case USB_AUDIO_IF_STREAM_TO_HOST:
		streamingToHost = streaming;
		if (streaming) {
			// Nothing is written to the FIFO yet, so the hardware answers every IN token with a zero-length packet
			usb_cstd_set_buf(USB_NULL, USB_CFG_PAUDIO_ISO_TO_HOST);
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
	return streamingToHost;
}

} // namespace deluge::io::usb::audio

using namespace deluge::io::usb::audio;

extern "C" void usbAudioPeripheralOpening(uint8_t enabled) {
	streamingFromHost = false;
	streamingToHost = false;
	usb_pstd_set_class_request_handler(enabled ? &handleClassRequest : nullptr);
	usb_pstd_set_interface_changed_handler(enabled ? &handleInterfaceChanged : nullptr);
}

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

#pragma once

#include <stdint.h>

// Layout of the composite USB Audio Class 2.0 + USB-MIDI configuration, used when USB audio is enabled.
// Everything is one function under a single interface association (IAD):
//
//   Interface 0: AudioControl - one fixed 44.1kHz clock, and two terminal pairs:
//                  computer -> Deluge: USB streaming IT -> line OT
//                  Deluge -> computer: line IT -> USB streaming OT
//   Interface 1: AudioStreaming, computer -> Deluge. Alt 0 idle, alt 1 stereo 24-bit, ISO OUT, adaptive
//   Interface 2: AudioStreaming, Deluge -> computer. Alt 0 idle, alt 1 stereo 24-bit, ISO IN, asynchronous
//   Interface 3: MIDIStreaming - identical to the MIDI-only configuration
//
// Only PIPE1 and PIPE2 can be isochronous, so there is no room for an explicit feedback endpoint. Windows doesn't
// support implicit feedback, so the OUT stream is adaptive (the computer sends at its own rate and the Deluge
// resamples). The IN stream is asynchronous - the Deluge's codec clock is the master.

#define USB_AUDIO_SAMPLE_RATE 44100
#define USB_AUDIO_NUM_CHANNELS 2
#define USB_AUDIO_SUBSLOT_SIZE 3 // Bytes per sample
#define USB_AUDIO_BIT_RESOLUTION 24
#define USB_AUDIO_BYTES_PER_FRAME (USB_AUDIO_NUM_CHANNELS * USB_AUDIO_SUBSLOT_SIZE)
// 44.1 frames per 1ms USB frame on average, so packets carry 44 or 45. One frame of headroom on top.
#define USB_AUDIO_MAX_FRAMES_PER_PACKET 46
#define USB_AUDIO_MAX_PACKET_SIZE (USB_AUDIO_MAX_FRAMES_PER_PACKET * USB_AUDIO_BYTES_PER_FRAME)

// Interface numbers
#define USB_AUDIO_IF_CONTROL 0
#define USB_AUDIO_IF_STREAM_FROM_HOST 1
#define USB_AUDIO_IF_STREAM_TO_HOST 2
#define USB_AUDIO_IF_MIDI 3
#define USB_AUDIO_NUM_INTERFACES 4

// Entity IDs within the AudioControl interface
#define USB_AUDIO_ID_CLOCK 1
#define USB_AUDIO_ID_FROM_HOST_USB_IT 2
#define USB_AUDIO_ID_FROM_HOST_LINE_OT 3
#define USB_AUDIO_ID_TO_HOST_LINE_IT 4
#define USB_AUDIO_ID_TO_HOST_USB_OT 5

// Endpoint numbers. MIDI has EP2 OUT and EP1 IN.
#define USB_AUDIO_EP_FROM_HOST 3 // OUT
#define USB_AUDIO_EP_TO_HOST 4   // IN

// UAC2 class-specific request codes and control selectors (ADC-2 appendix A)
#define UAC2_REQUEST_CUR 0x01
#define UAC2_REQUEST_RANGE 0x02
#define UAC2_CS_SAM_FREQ_CONTROL 0x01
#define UAC2_CS_CLOCK_VALID_CONTROL 0x02

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t g_audio_device[];
extern uint8_t g_audio_configuration[];
extern uint16_t g_usb_pstd_eptbl_audio[];

#ifdef __cplusplus
}
#endif

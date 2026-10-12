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

// USB Audio Class 2.0 device: stereo audio between the Deluge and a computer, alongside USB-MIDI. Only available
// when the Deluge is connected as a USB peripheral, with the USB Audio community feature on.

#ifdef __cplusplus
extern "C" {
#endif

// Called by openUSBPeripheral() before the USB driver is opened, to hook the audio function's class requests and
// stream starts/stops into the driver - or, if enabled is 0, to unhook them.
void usbAudioPeripheralOpening(uint8_t enabled);

#ifdef __cplusplus
}

#include <span>

struct StereoSample;

namespace deluge::io::usb::audio {

// Whether the computer has each stream open (i.e. has selected its streaming alternate setting)
bool isStreamingFromHost();
bool isStreamingToHost();

// Called by the audio engine with each block of frames it has just sent to the codec, exactly as sent
void feedOutputToHost(std::span<const StereoSample> frames);

// Whether the audio input comes wholly or partly from the computer, going by the USB Audio Input setting. If it does,
// the input is stereo, as if the line input were plugged in.
bool isInputFromHostSelected();

// Tells USB audio whether anything is plugged into the line or mic inputs. If not, "Line in + USB" just uses USB,
// rather than mixing in the internal mic.
void setExternalInputPluggedIn(bool pluggedIn);

// Called by the audio engine before each render, which reads numFramesToRead frames of input from i2sRXBufferPos.
// Makes sure those frames have the computer's audio in place of what the codec captured, or mixed in, per the USB Audio
// Input setting.
void processInputFromHost(size_t numFramesToRead);

} // namespace deluge::io::usb::audio
#endif

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

// The RZ/A1 only allows PIPE1 and PIPE2 to be isochronous, so USB audio gets both (and peripheral MIDI is kept off
// them - see r_usb_pmidi_config.h)
#define USB_CFG_PAUDIO_ISO_FROM_HOST (USB_PIPE2) // Receives audio from the computer
#define USB_CFG_PAUDIO_ISO_TO_HOST (USB_PIPE1)   // Sends audio to the computer

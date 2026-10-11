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

// Hooks for the Deluge into the peripheral side of the Renesas USB driver, beyond what it offers itself. Kept apart
// from r_usb_extern.h because, unlike that, this is safe to include from C++.

#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/* r_usb_preg_abs.c */
// Handler for a pipe serviced directly in the USB ISR rather than via the PCD task. event is USB_INT_BRDY,
// USB_INT_BEMP or USB_INT_NRDY. Runs in interrupt context, so must be quick.
typedef void (*usb_pstd_isr_pipe_handler_t)(uint16_t pipe, uint16_t event);
void usb_pstd_set_isr_pipe_handler(uint16_t pipe, usb_pstd_isr_pipe_handler_t handler);

/* r_usb_pstdrequest.c */
// Handler for class requests, so the Deluge can answer them itself (e.g. USB audio). Called at the start of the data
// stage for requests that have one - it should then call usb_pstd_ctrl_read() or usb_pstd_ctrl_write() - and at the
// status stage for requests that don't. Returns USB_TRUE if it dealt with the request, or USB_FALSE to STALL it.
typedef uint16_t (*usb_pstd_class_request_handler_t)(usb_setup_t* req, uint16_t ctsq);
void usb_pstd_set_class_request_handler(usb_pstd_class_request_handler_t handler);

// Called whenever an interface's alternate setting has been selected, by SET_INTERFACE, or by SET_CONFIGURATION
// putting every interface back to alternate setting 0
typedef void (*usb_pstd_interface_changed_handler_t)(uint16_t int_num, uint16_t alt_num);
void usb_pstd_set_interface_changed_handler(usb_pstd_interface_changed_handler_t handler);

#ifdef __cplusplus
}
#endif

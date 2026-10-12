/*
 * Copyright © 2018-2023 Synthstrom Audible Limited
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

// The USB-MIDI interface, shared by the MIDI-only configuration (r_usb_pmidi_descriptor.c) and the composite
// audio + MIDI configuration (deluge/io/usb/usb_audio_descriptor.c), so MIDI looks identical in both.

#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"

#define USB_BCDNUM (0x0200u) /* bcdUSB */
#define USB_CONFIGNUM (1u)   /* Configuration number */
#define USB_DCPMAXP (64u)    /* DCP max packet size */
// Get VID from
// http://www.mcselec.com/index.php?page=shop.product_details&flypage=shop.flypage&product_id=92&category_id=20&option=com_phpshop&Itemid=1
#define USB_VENDORID (0x16D0)  /* Vendor ID */
#define USB_PRODUCTID (0x0CE2) /* Product ID */

// size of the USB-MIDI interface, including its interface descriptor
/* 9 - interface description
 * 7 - MIDI streaming header
 * (6+9)*ncables - in and out jack per cable
 * (9+4+ncables)*2 - shared bulk endpoint, descriptors include list of cables
 */
#define NCABLES 3
#define USB_MIDI_CD_WTOTALLENGTH (9 + 7 + (15 * NCABLES) + (13 + NCABLES) * 2)

// USB midi defines
#define CS_INTERFACE 0x24
#define CS_ENDPOINT 0x25
#define MIDI_IN_JACK 0x02
#define MIDI_OUT_JACK 0x03

/* Interface Descriptor
 * 3rd level of USB declarations. This declares a single USB midi interface with 2 endpoints (subclass of audio)
 *
 * Midi Streaming Interface descriptors
 * A level below interface, specific to USB midi
 * ref - https://www.usb.org/sites/default/files/midi10.pdf sect 6.1.2
 * to add ports/cables add more jacks here
 *
 * MidiStreaming Endpoint Descriptors - USBMidi spec 6.2.1
 * These endpoints are shared across all jacks
 */
#define USB_MIDI_INTERFACE_DESCRIPTORS(interfaceNumber) USB_MIDI_INTERFACE_DESCRIPTORS_MAXP((interfaceNumber), 64)

// The same, with the bulk endpoints' max packet size - 64 at Full Speed, 512 at High Speed (USB 2.0 5.8.3)
#define USB_MIDI_INTERFACE_DESCRIPTORS_MAXP(interfaceNumber, maxPacketSize)                                            \
	USB_ID_BLENGTH,        /*  0:bLength */                                                                            \
	    USB_DT_INTERFACE,  /*  1:bDescriptorType */                                                                    \
	    (interfaceNumber), /*  2:bInterfaceNumber */                                                                   \
	    0,                 /*  3:bAlternateSetting */                                                                  \
	    2,                 /*  4:bNumEndpoints */                                                                      \
	    USB_IFCLS_AUD,     /*  5:bInterfaceClass(AUD) */                                                               \
	    0x03,              /*  6:bInterfaceSubClass(MIDI) */                                                           \
	    0,                 /*  7:bInterfaceProtocol */                                                                 \
	    0,                 /*  8:iInterface */                                                                         \
                                                                                                                       \
	    /* Header */                                                                                                   \
	    0x07,                        /* header length */                                                               \
	    CS_INTERFACE,                /* bDescriptorType - CS interface */                                              \
	    0x01,                        /* Subtype - Midi Streaming Header */                                             \
	    0x00, 0x01,                  /* BCD revision (1.00) */                                                         \
	    (uint8_t)(7 + 15 * NCABLES), /* TotalLength - LSB of interface descriptors - 7+15*ncables */                   \
	    0x00,                        /* Interface descriptors MSB */                                                   \
                                                                                                                       \
	    /* MIDI_IN 1 */                                                                                                \
	    0x06,         /* bLength */                                                                                    \
	    CS_INTERFACE, /* bDescriptorType */                                                                            \
	    MIDI_IN_JACK, /* bDescriptorSubtype - MIDI_IN_JACK */                                                          \
	    0x01,         /* bJackType - EMBEDDED */                                                                       \
	    0x01,         /* bJackID - 1 */                                                                                \
	    0x00,         /* iJack (unused) */                                                                             \
                                                                                                                       \
	    /* MIDI_OUT 1 */                                                                                               \
	    0x09,          /* bLength */                                                                                   \
	    CS_INTERFACE,  /* bDescriptorType - CS_I */                                                                    \
	    MIDI_OUT_JACK, /* bDescriptorSubtype - MIDI_OUT_JACK */                                                        \
	    0x01,          /* bJackType - EMBEDDED */                                                                      \
	    0x02,          /* bJackID - 2 */                                                                               \
	    0x01,          /* bNrInputPins */                                                                              \
	    0x01,          /* BaSourceID - the midi in jack associated with this one */                                    \
	    0x01,          /* BaSourcePin */                                                                               \
	    0x00,          /* iJack (unused) */                                                                            \
                                                                                                                       \
	    /* MIDI_IN 2 */                                                                                                \
	    0x06,         /* bLength */                                                                                    \
	    CS_INTERFACE, /* bDescriptorType */                                                                            \
	    MIDI_IN_JACK, /* bDescriptorSubtype - MIDI_IN_JACK */                                                          \
	    0x01,         /* bJackType - EMBEDDED */                                                                       \
	    0x03,         /* bJackID */                                                                                    \
	    0x00,         /* iJack (unused) */                                                                             \
                                                                                                                       \
	    /* MIDI_OUT 2 */                                                                                               \
	    0x09,          /* bLength */                                                                                   \
	    CS_INTERFACE,  /* bDescriptorType - CS_I */                                                                    \
	    MIDI_OUT_JACK, /* bDescriptorSubtype - MIDI_OUT_JACK */                                                        \
	    0x01,          /* bJackType - EMBEDDED */                                                                      \
	    0x04,          /* bJackID */                                                                                   \
	    0x01,          /* bNrInputPins */                                                                              \
	    0x02,          /* BaSourceID */                                                                                \
	    0x01,          /* BaSourcePin */                                                                               \
	    0x00,          /* iJack (unused) */                                                                            \
                                                                                                                       \
	    /* MIDI_IN 3 */                                                                                                \
	    0x06,         /* bLength */                                                                                    \
	    CS_INTERFACE, /* bDescriptorType */                                                                            \
	    MIDI_IN_JACK, /* bDescriptorSubtype - MIDI_IN_JACK */                                                          \
	    0x01,         /* bJackType - EMBEDDED */                                                                       \
	    0x05,         /* bJackID */                                                                                    \
	    0x00,         /* iJack (unused) */                                                                             \
                                                                                                                       \
	    /* MIDI_OUT 3 */                                                                                               \
	    0x09,          /* bLength */                                                                                   \
	    CS_INTERFACE,  /* bDescriptorType - CS_I */                                                                    \
	    MIDI_OUT_JACK, /* bDescriptorSubtype - MIDI_OUT_JACK */                                                        \
	    0x01,          /* bJackType - EMBEDDED */                                                                      \
	    0x06,          /* bJackID */                                                                                   \
	    0x01,          /* bNrInputPins */                                                                              \
	    0x05,          /* BaSourceID */                                                                                \
	    0x01,          /* BaSourcePin */                                                                               \
	    0x00,          /* iJack (unused) */                                                                            \
                                                                                                                       \
	    /* 28 bytes for bulk endpoints */ /* USB standard bulk out */                                                  \
	    0x09,                             /* bLength */                                                                \
	    0x05,                             /* bDescriptorType = ENDPOINT */                                             \
	    (uint8_t)(USB_EP_OUT | USB_EP2),  /* bEndpointAddress. D7 direction, low 4 addr */                             \
	    0x02,                             /* bmAttributes (bulk) */                                                    \
	    (uint8_t)((maxPacketSize) & 0xFF), (uint8_t)((maxPacketSize) >> 8), /* wMaxPacketSize */                       \
	    0x00,                                                               /* bInterval */                            \
	    0x00,                                                               /* bRefresh */                             \
	    0x00, /* bSynchAddress */                                           /* midi class specific bulk out */         \
	    (uint8_t)(4 + NCABLES),                                             /* bLength - 4+ncables */                  \
	    CS_ENDPOINT,                                                        /* bDescriptorType - CS_ENDPOINT */        \
	    0x01,                                                               /* bDescriptorSubType - MS_GENERAL */      \
	    0x03,       /* bNumEmbMidiJack - number of MIDI IN jacks */                                                    \
	    0x01,       /* BaAssocJackID - ID of first associated jack */                                                  \
	    0x03, 0x05, /* ID of last associated jack */                                                                   \
                                                                                                                       \
	    /* USB standard bulk in - same fields as above, differences annotated */                                       \
	    0x09,                                                               /* bLength */                              \
	    0x05,                                                               /* bDescriptor */                          \
	    (uint8_t)(USB_EP_IN | USB_EP1),                                     /* different address */                    \
	    0x02,                                                               /* bmAttributes */                         \
	    (uint8_t)((maxPacketSize) & 0xFF), (uint8_t)((maxPacketSize) >> 8), /* wMaxPacketSize */                       \
	    0x00,                                                               /* bInterval */                            \
	    0x00,                                                               /* bRefresh */                             \
	    0x00, /* bSynchAddress */                                           /* midi specific bulk in */                \
	    (uint8_t)(4 + NCABLES),                                             /* bLength - 4+ncables */                  \
	    CS_ENDPOINT,                                                        /* bDescriptorType - CS_ENDPOINT */        \
	    0x01,                                                               /* bDescriptorSubtype */                   \
	    0x03,      /* bNumEmbMidiJack - number of MIDI OUT jacks */                                                    \
	    0x02,      /* BaAssocJackID - first associated jack */                                                         \
	    0x04, 0x06 /* Last associated jack */

extern uint8_t* g_midi_string_table[];

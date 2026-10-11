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

// Descriptors for the composite USB Audio Class 2.0 + USB-MIDI configuration. See usb_audio_descriptor.h for the
// layout. Refs: USB Device Class Definition for Audio Devices 2.0 (ADC-2) and Audio Data Formats 2.0 (FMT-2).

#include "deluge/io/usb/usb_audio_descriptor.h"
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"
#include "deluge/drivers/usb/userdef/r_usb_pmidi_config.h"
#include "deluge/io/midi/r_usb_pmidi_descriptor.h"

// Differs from the MIDI-only configuration's, so hosts that cache descriptors per VID/PID/bcdDevice (Windows) don't
// mix the two configurations up. Bump it whenever the audio descriptors change, for the same reason.
#define USB_AUDIO_RELEASE (0x0202u)

#define LO(x) (uint8_t)((x) & 0xFF)
#define HI(x) (uint8_t)(((x) >> 8) & 0xFF)
#define LE16(x) LO(x), HI(x)
#define LE32(x) LO(x), HI(x), (uint8_t)(((x) >> 16) & 0xFF), (uint8_t)(((x) >> 24) & 0xFF)

// Descriptor types and subtypes (ADC-2 appendix A)
#define DT_INTERFACE_ASSOCIATION 0x0B
#define CS_AC_HEADER 0x01
#define CS_AC_INPUT_TERMINAL 0x02
#define CS_AC_OUTPUT_TERMINAL 0x03
#define CS_AC_CLOCK_SOURCE 0x0A
#define CS_AS_GENERAL 0x01
#define CS_AS_FORMAT_TYPE 0x02
#define CS_EP_GENERAL 0x01

#define AUDIO_SUBCLASS_CONTROL 0x01
#define AUDIO_SUBCLASS_STREAMING 0x02
#define AUDIO_PROTOCOL_IP_VERSION_02_00 0x20
#define AUDIO_CATEGORY_MUSICAL_INSTRUMENT 0x09

#define TERMINAL_USB_STREAMING 0x0101
#define TERMINAL_LINE_CONNECTOR 0x0603

#define CHANNEL_CONFIG_STEREO 0x00000003 // Front left, front right

#define EP_ATTR_ISO_SYNC 0x0D     // Isochronous, synchronous, data
#define EP_ATTR_ISO_ADAPTIVE 0x09 // Isochronous, adaptive, data

// Lengths
#define AC_HEADER_LENGTH 9
#define CLOCK_SOURCE_LENGTH 8
#define INPUT_TERMINAL_LENGTH 17
#define OUTPUT_TERMINAL_LENGTH 12
#define AC_TOTAL_LENGTH (AC_HEADER_LENGTH + CLOCK_SOURCE_LENGTH + 2 * (INPUT_TERMINAL_LENGTH + OUTPUT_TERMINAL_LENGTH))

#define AS_GENERAL_LENGTH 16
#define FORMAT_TYPE_I_LENGTH 6
#define ISO_ENDPOINT_LENGTH 7
#define CS_ISO_ENDPOINT_LENGTH 8
#define AS_INTERFACE_TOTAL_LENGTH                                                                                      \
	(2 * USB_ID_BLENGTH + AS_GENERAL_LENGTH + FORMAT_TYPE_I_LENGTH + ISO_ENDPOINT_LENGTH + CS_ISO_ENDPOINT_LENGTH)

#define IAD_LENGTH 8
#define AUDIO_CONFIG_TOTAL_LENGTH                                                                                      \
	(USB_CD_BLENGTH + IAD_LENGTH + USB_ID_BLENGTH + AC_TOTAL_LENGTH + 2 * AS_INTERFACE_TOTAL_LENGTH                    \
	 + USB_MIDI_CD_WTOTALLENGTH)

uint8_t g_audio_device[USB_DD_BLENGTH + (USB_DD_BLENGTH % 2)] = {
    USB_DD_BLENGTH,          /*  0:bLength */
    USB_DT_DEVICE,           /*  1:bDescriptorType */
    LE16(USB_BCDNUM),        /*  2:bcdUSB */
    0xEF,                    /*  4:bDeviceClass - Miscellaneous */
    0x02,                    /*  5:bDeviceSubClass - Common class */
    0x01,                    /*  6:bDeviceProtocol - Interface association descriptor */
    (uint8_t)USB_DCPMAXP,    /*  7:bMAXPacketSize(for DCP) */
    LE16(USB_VENDORID),      /*  8:idVendor */
    LE16(USB_PRODUCTID),     /* 10:idProduct */
    LE16(USB_AUDIO_RELEASE), /* 12:bcdDevice */
    1,                       /* 14:iManufacturer */
    2,                       /* 15:iProduct */
    0,                       /* 16:iSerialNumber */
    USB_CONFIGNUM,           /* 17:bNumConfigurations */
};

// Standard and class-specific descriptors for one AudioStreaming interface: an idle alt 0, and an alt 1 carrying
// stereo 24-bit PCM over one isochronous endpoint
#define AUDIO_STREAMING_INTERFACE(interfaceNumber, terminalLink, endpointAddress, endpointAttributes)                  \
	/* Alt 0 - no endpoints, so no bandwidth */                                                                        \
	USB_ID_BLENGTH, USB_DT_INTERFACE, (interfaceNumber), 0 /* bAlternateSetting */, 0 /* bNumEndpoints */,             \
	    USB_IFCLS_AUD, AUDIO_SUBCLASS_STREAMING, AUDIO_PROTOCOL_IP_VERSION_02_00, 0 /* iInterface */,                  \
                                                                                                                       \
	    /* Alt 1 - streaming */                                                                                        \
	    USB_ID_BLENGTH, USB_DT_INTERFACE, (interfaceNumber), 1 /* bAlternateSetting */, 1 /* bNumEndpoints */,         \
	    USB_IFCLS_AUD, AUDIO_SUBCLASS_STREAMING, AUDIO_PROTOCOL_IP_VERSION_02_00, 0 /* iInterface */,                  \
                                                                                                                       \
	    /* Class-specific AS interface (ADC-2 4.9.2) */                                                                \
	    AS_GENERAL_LENGTH, CS_INTERFACE, CS_AS_GENERAL, (terminalLink), 0x00 /* bmControls */,                         \
	    0x01 /* bFormatType - FORMAT_TYPE_I */, LE32(0x00000001) /* bmFormats - PCM */, USB_AUDIO_NUM_CHANNELS,        \
	    LE32(CHANNEL_CONFIG_STEREO), 0 /* iChannelNames */,                                                            \
                                                                                                                       \
	    /* Type I format (FMT-2 2.3.1.6) */                                                                            \
	    FORMAT_TYPE_I_LENGTH, CS_INTERFACE, CS_AS_FORMAT_TYPE, 0x01 /* bFormatType - FORMAT_TYPE_I */,                 \
	    USB_AUDIO_SUBSLOT_SIZE, USB_AUDIO_BIT_RESOLUTION,                                                              \
                                                                                                                       \
	    /* Standard isochronous endpoint (ADC-2 4.10.1.1) */                                                           \
	    ISO_ENDPOINT_LENGTH, USB_DT_ENDPOINT, (endpointAddress), (endpointAttributes),                                 \
	    LE16(USB_AUDIO_MAX_PACKET_SIZE), 1 /* bInterval - every 1ms frame */,                                          \
                                                                                                                       \
	    /* Class-specific isochronous endpoint (ADC-2 4.10.1.2) */                                                     \
	    CS_ISO_ENDPOINT_LENGTH, CS_ENDPOINT, CS_EP_GENERAL, 0x00 /* bmAttributes */, 0x00 /* bmControls */,            \
	    0x00 /* bLockDelayUnits */, LE16(0) /* wLockDelay */

uint8_t g_audio_configuration[AUDIO_CONFIG_TOTAL_LENGTH + (AUDIO_CONFIG_TOTAL_LENGTH % 2)] = {
    USB_CD_BLENGTH,                  /* bLength */
    USB_DT_CONFIGURATION,            /* bDescriptorType */
    LE16(AUDIO_CONFIG_TOTAL_LENGTH), /* wTotalLength */
    USB_AUDIO_NUM_INTERFACES,        /* bNumInterfaces */
    1,                               /* bConfigurationValue */
    0,                               /* iConfiguration */
    (uint8_t)(USB_CF_RESERVED),      /* bmAttributes */
    (uint8_t)(500 / 2),              /* bMaxPower (2mA unit) */

    /* Interface association - groups all the interfaces into one audio function, MIDI included. Windows then binds
     * the MIDI interface to its USB-MIDI driver as a separate function, and Linux and macOS find it via the IAD. */
    IAD_LENGTH,                      /* bLength */
    DT_INTERFACE_ASSOCIATION,        /* bDescriptorType */
    USB_AUDIO_IF_CONTROL,            /* bFirstInterface */
    USB_AUDIO_NUM_INTERFACES,        /* bInterfaceCount */
    USB_IFCLS_AUD,                   /* bFunctionClass */
    0x00,                            /* bFunctionSubClass - undefined */
    AUDIO_PROTOCOL_IP_VERSION_02_00, /* bFunctionProtocol */
    2,                               /* iFunction - "Deluge" */

    /* AudioControl interface */
    USB_ID_BLENGTH,                  /* bLength */
    USB_DT_INTERFACE,                /* bDescriptorType */
    USB_AUDIO_IF_CONTROL,            /* bInterfaceNumber */
    0,                               /* bAlternateSetting */
    0,                               /* bNumEndpoints */
    USB_IFCLS_AUD,                   /* bInterfaceClass */
    AUDIO_SUBCLASS_CONTROL,          /* bInterfaceSubClass */
    AUDIO_PROTOCOL_IP_VERSION_02_00, /* bInterfaceProtocol */
    0,                               /* iInterface */

    /* Class-specific AC interface header (ADC-2 4.7.2) */
    AC_HEADER_LENGTH,
    CS_INTERFACE,
    CS_AC_HEADER,
    LE16(0x0200) /* bcdADC */,
    AUDIO_CATEGORY_MUSICAL_INSTRUMENT,
    LE16(AC_TOTAL_LENGTH),
    0x00 /* bmControls */,

    /* Clock source (ADC-2 4.7.2.1) - the codec's crystal. Internal fixed clock, so frequency and validity are
     * read-only. */
    CLOCK_SOURCE_LENGTH,
    CS_INTERFACE,
    CS_AC_CLOCK_SOURCE,
    USB_AUDIO_ID_CLOCK,
    0x01 /* bmAttributes - internal fixed */,
    0x05 /* bmControls - frequency read-only, validity read-only */,
    0 /* bAssocTerminal */,
    0 /* iClockSource */,

    /* Computer -> Deluge: USB streaming input terminal (ADC-2 4.7.2.4) */
    INPUT_TERMINAL_LENGTH,
    CS_INTERFACE,
    CS_AC_INPUT_TERMINAL,
    USB_AUDIO_ID_FROM_HOST_USB_IT,
    LE16(TERMINAL_USB_STREAMING),
    0 /* bAssocTerminal */,
    USB_AUDIO_ID_CLOCK,
    USB_AUDIO_NUM_CHANNELS,
    LE32(CHANNEL_CONFIG_STEREO),
    0 /* iChannelNames */,
    LE16(0x0000) /* bmControls */,
    0 /* iTerminal */,

    /* ... to a line output terminal (ADC-2 4.7.2.5) */
    OUTPUT_TERMINAL_LENGTH,
    CS_INTERFACE,
    CS_AC_OUTPUT_TERMINAL,
    USB_AUDIO_ID_FROM_HOST_LINE_OT,
    LE16(TERMINAL_LINE_CONNECTOR),
    0 /* bAssocTerminal */,
    USB_AUDIO_ID_FROM_HOST_USB_IT /* bSourceID */,
    USB_AUDIO_ID_CLOCK,
    LE16(0x0000) /* bmControls */,
    0 /* iTerminal */,

    /* Deluge -> computer: line input terminal */
    INPUT_TERMINAL_LENGTH,
    CS_INTERFACE,
    CS_AC_INPUT_TERMINAL,
    USB_AUDIO_ID_TO_HOST_LINE_IT,
    LE16(TERMINAL_LINE_CONNECTOR),
    0 /* bAssocTerminal */,
    USB_AUDIO_ID_CLOCK,
    USB_AUDIO_NUM_CHANNELS,
    LE32(CHANNEL_CONFIG_STEREO),
    0 /* iChannelNames */,
    LE16(0x0000) /* bmControls */,
    0 /* iTerminal */,

    /* ... to a USB streaming output terminal */
    OUTPUT_TERMINAL_LENGTH,
    CS_INTERFACE,
    CS_AC_OUTPUT_TERMINAL,
    USB_AUDIO_ID_TO_HOST_USB_OT,
    LE16(TERMINAL_USB_STREAMING),
    0 /* bAssocTerminal */,
    USB_AUDIO_ID_TO_HOST_LINE_IT /* bSourceID */,
    USB_AUDIO_ID_CLOCK,
    LE16(0x0000) /* bmControls */,
    0 /* iTerminal */,

    AUDIO_STREAMING_INTERFACE(USB_AUDIO_IF_STREAM_FROM_HOST, USB_AUDIO_ID_FROM_HOST_USB_IT,
                              (uint8_t)(USB_EP_OUT | USB_AUDIO_EP_FROM_HOST), EP_ATTR_ISO_ADAPTIVE),

    AUDIO_STREAMING_INTERFACE(USB_AUDIO_IF_STREAM_TO_HOST, USB_AUDIO_ID_TO_HOST_USB_OT,
                              (uint8_t)(USB_EP_IN | USB_AUDIO_EP_TO_HOST), EP_ATTR_ISO_SYNC),

    USB_MIDI_INTERFACE_DESCRIPTORS(USB_AUDIO_IF_MIDI),
};

// Pipe table for the configuration above. usb_peri_pipe_info() fills in one row per endpoint descriptor, in the order
// they appear in the configuration, so the rows must be in that order too: audio from the computer, audio to the
// computer, then MIDI's two (as in g_usb_pstd_eptbl).
// Buffer memory is in 64-byte blocks. 0-7 are taken by the DCP and PIPE6-9, and MIDI's pipes use 8-23 and 72-87.
#define ISO_PIPE_BUFFER_SIZE 320u // USB_AUDIO_MAX_PACKET_SIZE rounded up to 64 bytes. Double-buffered, so 10 blocks.
uint16_t g_usb_pstd_eptbl_audio[] = {
    USB_CFG_PAUDIO_ISO_FROM_HOST, /* Pipe No. */
    /* TYPE    / BFRE        / DBLB         / CNTMD         / SHTNAK   / DIR      / EPNUM */
    USB_NULL | USB_BFREOFF | USB_CFG_DBLB | USB_CFG_CNTMD | USB_NULL | USB_NULL | USB_NULL, /* PIPECFG */
    (uint16_t)USB_BUF_SIZE(ISO_PIPE_BUFFER_SIZE) | USB_BUF_NUMB(24u),                       /* PIPEBUF */
    USB_NULL,                                                                               /* PIPEMAXP */
    USB_NULL,                                                                               /* PIPEPERI */
    USB_NULL,                                                                               /* reserve */

    USB_CFG_PAUDIO_ISO_TO_HOST, /* Pipe No. */
    /* TYPE    / BFRE        / DBLB         / CNTMD         / SHTNAK   / DIR      / EPNUM */
    USB_NULL | USB_BFREOFF | USB_CFG_DBLB | USB_CFG_CNTMD | USB_NULL | USB_NULL | USB_NULL, /* PIPECFG */
    (uint16_t)USB_BUF_SIZE(ISO_PIPE_BUFFER_SIZE) | USB_BUF_NUMB(40u),                       /* PIPEBUF */
    USB_NULL,                                                                               /* PIPEMAXP */
    USB_IFISON, /* PIPEPERI - flush data the host didn't collect in its frame, so it never goes out late */
    USB_NULL,   /* reserve */

    USB_CFG_PMIDI_BULK_IN, /* Pipe No. */
    /* TYPE    / BFRE        / DBLB         / CNTMD         / SHTNAK         / DIR      / EPNUM */
    USB_NULL | USB_BFREOFF | USB_CFG_DBLB | USB_CFG_CNTMD | USB_CFG_SHTNAK | USB_NULL | USB_NULL, /* PIPECFG */
    (uint16_t)USB_BUF_SIZE(512u) | USB_BUF_NUMB(8u),                                              /* PIPEBUF */
    USB_NULL,                                                                                     /* PIPEMAXP */
    USB_NULL,                                                                                     /* PIPEPERI */
    USB_NULL,                                                                                     /* reserve */

    USB_CFG_PMIDI_BULK_OUT, /* Pipe No. */
    /* TYPE    / BFRE        / DBLB         / CNTMD         / SHTNAK         / DIR      / EPNUM */
    USB_NULL | USB_BFREOFF | USB_CFG_DBLB | USB_CFG_CNTMD | USB_CFG_SHTNAK | USB_NULL | USB_NULL, /* PIPECFG */
    (uint16_t)USB_BUF_SIZE(512u) | USB_BUF_NUMB(72u),                                             /* PIPEBUF */
    USB_NULL,                                                                                     /* PIPEMAXP */
    USB_NULL,                                                                                     /* PIPEPERI */
    USB_NULL,                                                                                     /* reserve */

    /* Pipe end */
    USB_PDTBLEND,
};

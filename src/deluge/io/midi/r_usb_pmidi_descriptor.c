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

/*
 * Portions of this code initially copied from the relevant sample code by Renesas.
 *
 * These global config definitions are used in USBConnection.c
 *
 * The connect functions in USBConnection.c are called around line 800 of deluge_main
 */

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "deluge/io/midi/r_usb_pmidi_descriptor.h"

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define USB_RELEASE (0x0200u) /* Release Number */

// 9 for config descriptor. Add any additional config lengths here
#define TOTAL_CONFIG_LENGTH (9 + USB_MIDI_CD_WTOTALLENGTH)
// Good summary ref on overall USB structure https://www.beyondlogic.org/usbnutshell/usb5.shtml

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/
/* Standard Device Descriptor
 * Top level USB descriptor
 * Used by host to enumerate device - e.g. what is this
 * bytes 4-7 declares that the device is specified at the interface level
 * bytes 8-16 declare that it's a synthstrom deluge (presumably)
 * byte 17 says how many configurations are available - e.g. it could
 * offer both an FS and HS configuration and allow the host to choose,
 * but in this case we offer a single USB midi FS config
 */
uint8_t g_midi_device[USB_DD_BLENGTH + (USB_DD_BLENGTH % 2)] = {
    USB_DD_BLENGTH,                                        /*  0:bLength */
    USB_DT_DEVICE,                                         /*  1:bDescriptorType */
    (uint8_t)(USB_BCDNUM&(uint8_t)0xff),                   /*  2:bcdUSB_lo */
    (uint8_t)((uint8_t)(USB_BCDNUM >> 8) & (uint8_t)0xff), /*  3:bcdUSB_hi */
                                                           // Device to be specified at interface level
    0x00,                                                  /*  4:bDeviceClass */
    0x00,                                                  /*  5:bDeviceSubClass */
    0x00,                                                  /*  6:bDeviceProtocol */

    (uint8_t)USB_DCPMAXP,                                     /*  7:bMAXPacketSize(for DCP) */
    (uint8_t)(USB_VENDORID&(uint8_t)0xff),                    /*  8:idVendor_lo */
    (uint8_t)((uint8_t)(USB_VENDORID >> 8) & (uint8_t)0xff),  /*  9:idVendor_hi */
    (uint8_t)(USB_PRODUCTID&(uint8_t)0xff),                   /* 10:idProduct_lo */
    (uint8_t)((uint8_t)(USB_PRODUCTID >> 8) & (uint8_t)0xff), /* 11:idProduct_hi */
    (uint8_t)(USB_RELEASE&(uint8_t)0xff),                     /* 12:bcdDevice_lo */
    (uint8_t)((uint8_t)(USB_RELEASE >> 8) & (uint8_t)0xff),   /* 13:bcdDevice_hi */
    1,                                                        /* 14:iManufacturer */
    2,                                                        /* 15:iProduct */
    0,                                                        /* 16:iSerialNumber */
    USB_CONFIGNUM,                                            /* 17:bNumConfigurations */
};

/************************************************************
 * Configuration Or Other_Speed_Configuration Descriptor     *
 ************************************************************/

/* USB Configuration description - USB spec 9.6.3
 * 2nd level of USB declaration - defines power and number of interfaces
 * This config specifies that it is configuration 1 and has a single interface.
 * The composite USB audio + MIDI configuration is in deluge/io/usb/usb_audio_descriptor.c
 */
uint8_t g_midi_configuration[TOTAL_CONFIG_LENGTH + (TOTAL_CONFIG_LENGTH % 2)] = {
    USB_CD_BLENGTH,                       /*  0:bLength */
    USB_DT_CONFIGURATION,                 /*  1:bDescriptorType */
    (uint8_t)(TOTAL_CONFIG_LENGTH % 256), /*  2:wTotalLength(L) */
    (uint8_t)(TOTAL_CONFIG_LENGTH / 256), /*  3:wTotalLength(H) */
    1,                                    /*  4:bNumInterfaces */
    1,                                    /*  5:bConfigurationValue */
    0,                                    /*  6:iConfiguration */
    (uint8_t)(USB_CF_RESERVED),           /*  7:bmAttributes */
    (uint8_t)(500 / 2),                   /*  8:bMaxPower (2mA unit) */

    USB_MIDI_INTERFACE_DESCRIPTORS(0),
};

/*************************************
 *    String Descriptor               *
 *************************************/
uint8_t g_midi_string0[] = {
    /* UNICODE 0x0409 English (United States) */
    4,             /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    0x09, 0x04     /*  2:wLANGID[0] */
};

uint8_t g_midi_string1[] = {
    38,            /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'S',
    0x00, /*  2:bString */
    'y',
    0x00,
    'n',
    0x00,
    't',
    0x00,
    'h',
    0x00,
    's',
    0x00,
    't',
    0x00,
    'r',
    0x00,
    'o',
    0x00,
    'm',
    0x00,
    ' ',
    0x00,
    'A',
    0x00,
    'u',
    0x00,
    'd',
    0x00,
    'i',
    0x00,
    'b',
    0x00,
    'l',
    0x00,
    'e',
    0x00,
};

uint8_t g_midi_string2[] = {
    14,            /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'D',
    0x00, /*  2:bString */
    'e',
    0x00,
    'l',
    0x00,
    'u',
    0x00,
    'g',
    0x00,
    'e',
    0x00,
};

uint8_t g_midi_string3[] = {
    8,             /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'O',
    0x00, /*  2:bString */
    'U',
    0x00,
    'T',
    0x00,
};

uint8_t g_midi_string4[] = {
    6,             /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'I',
    0x00, /*  2:bString */
    'N',
    0x00,
};

uint8_t* g_midi_string_table[] = {g_midi_string0, g_midi_string1, g_midi_string2, g_midi_string3, g_midi_string4};

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

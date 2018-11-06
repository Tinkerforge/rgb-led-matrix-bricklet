/* rgb-led-matrix-bricklet
 * Copyright (C) 2017 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.h: TFP protocol message handling
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>

#include "bricklib2/bootloader/bootloader.h"

// Default functions
BootloaderHandleMessageResponse handle_message(const void *data, void *response);
void communication_tick(void);
void communication_init(void);

// Constants


// Function and callback IDs and structs
#define FID_SET_RED 1
#define FID_GET_RED 2
#define FID_SET_GREEN 3
#define FID_GET_GREEN 4
#define FID_SET_BLUE 5
#define FID_GET_BLUE 6
#define FID_SET_FRAME_DURATION 7
#define FID_GET_FRAME_DURATION 8
#define FID_DRAW_FRAME 9
#define FID_GET_SUPPLY_VOLTAGE 10

#define FID_CALLBACK_FRAME_STARTED 11

typedef struct {
	TFPMessageHeader header;
	uint8_t red[64];
} __attribute__((__packed__)) SetRed;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetRed;

typedef struct {
	TFPMessageHeader header;
	uint8_t red[64];
} __attribute__((__packed__)) GetRed_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t green[64];
} __attribute__((__packed__)) SetGreen;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetGreen;

typedef struct {
	TFPMessageHeader header;
	uint8_t green[64];
} __attribute__((__packed__)) GetGreen_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t blue[64];
} __attribute__((__packed__)) SetBlue;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetBlue;

typedef struct {
	TFPMessageHeader header;
	uint8_t blue[64];
} __attribute__((__packed__)) GetBlue_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t frame_duration;
} __attribute__((__packed__)) SetFrameDuration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetFrameDuration;

typedef struct {
	TFPMessageHeader header;
	uint16_t frame_duration;
} __attribute__((__packed__)) GetFrameDuration_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) DrawFrame;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSupplyVoltage;

typedef struct {
	TFPMessageHeader header;
	uint16_t voltage;
} __attribute__((__packed__)) GetSupplyVoltage_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t frame_number;
} __attribute__((__packed__)) FrameStarted_Callback;


// Function prototypes
BootloaderHandleMessageResponse set_red(const SetRed *data);
BootloaderHandleMessageResponse get_red(const GetRed *data, GetRed_Response *response);
BootloaderHandleMessageResponse set_green(const SetGreen *data);
BootloaderHandleMessageResponse get_green(const GetGreen *data, GetGreen_Response *response);
BootloaderHandleMessageResponse set_blue(const SetBlue *data);
BootloaderHandleMessageResponse get_blue(const GetBlue *data, GetBlue_Response *response);
BootloaderHandleMessageResponse set_frame_duration(const SetFrameDuration *data);
BootloaderHandleMessageResponse get_frame_duration(const GetFrameDuration *data, GetFrameDuration_Response *response);
BootloaderHandleMessageResponse draw_frame(const DrawFrame *data);
BootloaderHandleMessageResponse get_supply_voltage(const GetSupplyVoltage *data, GetSupplyVoltage_Response *response);

// Callbacks
bool handle_frame_started_callback(void);

#define COMMUNICATION_CALLBACK_TICK_WAIT_MS 1
#define COMMUNICATION_CALLBACK_HANDLER_NUM 1
#define COMMUNICATION_CALLBACK_LIST_INIT \
	handle_frame_started_callback, \


#endif

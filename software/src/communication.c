/* rgb-led-matrix-bricklet
 * Copyright (C) 2017 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.c: TFP protocol message handling
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

#include "communication.h"

#include "bricklib2/utility/communication_callback.h"
#include "bricklib2/protocols/tfp/tfp.h"
#include "matrix.h"

extern Matrix matrix;

BootloaderHandleMessageResponse handle_message(const void *message, void *response) {
	switch(tfp_get_fid_from_message(message)) {
		case FID_SET_RED: return set_red(message);
		case FID_GET_RED: return get_red(message, response);
		case FID_SET_GREEN: return set_green(message);
		case FID_GET_GREEN: return get_green(message, response);
		case FID_SET_BLUE: return set_blue(message);
		case FID_GET_BLUE: return get_blue(message, response);
		case FID_SET_FRAME_DURATION: return set_frame_duration(message);
		case FID_GET_FRAME_DURATION: return get_frame_duration(message, response);
		case FID_DRAW_FRAME: return draw_frame(message);
		case FID_GET_SUPPLY_VOLTAGE: return get_supply_voltage(message, response);
		default: return HANDLE_MESSAGE_RESPONSE_NOT_SUPPORTED;
	}
}


BootloaderHandleMessageResponse set_red(const SetRed *data) {
	memcpy(matrix.buffer_in.r, data->red, MATRIX_SIZE);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_red(const GetRed *data, GetRedResponse *response) {
	response->header.length = sizeof(GetRedResponse);
	memcpy(response->red, matrix.buffer_in.r, MATRIX_SIZE);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_green(const SetGreen *data) {
	memcpy(matrix.buffer_in.g, data->green, MATRIX_SIZE);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_green(const GetGreen *data, GetGreenResponse *response) {
	response->header.length = sizeof(GetGreenResponse);
	memcpy(response->green, matrix.buffer_in.g, MATRIX_SIZE);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_blue(const SetBlue *data) {
	memcpy(matrix.buffer_in.b, data->blue, MATRIX_SIZE);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_blue(const GetBlue *data, GetBlueResponse *response) {
	response->header.length = sizeof(GetBlueResponse);
	memcpy(response->blue, matrix.buffer_in.b, MATRIX_SIZE);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_frame_duration(const SetFrameDuration *data) {
	if(data->frame_duration > 0 && data->frame_duration < 10) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	matrix.frame_duration = data->frame_duration;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_frame_duration(const GetFrameDuration *data, GetFrameDurationResponse *response) {
	response->header.length  = sizeof(GetFrameDurationResponse);
	response->frame_duration = matrix.frame_duration;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse draw_frame(const DrawFrame *data) {
	if(matrix.frame_duration != 0) {
		matrix_draw_frame(&matrix);
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_supply_voltage(const GetSupplyVoltage *data, GetSupplyVoltageResponse *response) {
	response->header.length = sizeof(GetSupplyVoltageResponse);
	response->voltage       = 42; // TODO: Implement me!

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}




bool handle_frame_started_callback(void) {
	static bool is_buffered = false;
	static FrameStartedCallback cb;

	if(!is_buffered) {
		tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(FrameStartedCallback), FID_CALLBACK_FRAME_STARTED);
		if(matrix.frame_started) {
			cb.frame_number = matrix.frame_number;
			is_buffered = true;
			matrix.frame_started = false;
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(FrameStartedCallback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

void communication_tick(void) {
	communication_callback_tick();
}

void communication_init(void) {
	communication_callback_init();
}

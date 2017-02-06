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

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_red(const GetRed *data, GetRedResponse *response) {
	response->header.length = sizeof(GetRedResponse);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_green(const SetGreen *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_green(const GetGreen *data, GetGreenResponse *response) {
	response->header.length = sizeof(GetGreenResponse);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_blue(const SetBlue *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_blue(const GetBlue *data, GetBlueResponse *response) {
	response->header.length = sizeof(GetBlueResponse);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_frame_duration(const SetFrameDuration *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_frame_duration(const GetFrameDuration *data, GetFrameDurationResponse *response) {
	response->header.length = sizeof(GetFrameDurationResponse);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse draw_frame(const DrawFrame *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_supply_voltage(const GetSupplyVoltage *data, GetSupplyVoltageResponse *response) {
	response->header.length = sizeof(GetSupplyVoltageResponse);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}




bool handle_frame_started_callback(void) {
	static bool is_buffered = false;
	static FrameStartedCallback cb;

	if(!is_buffered) {
		tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(FrameStartedCallback), FID_CALLBACK_FRAME_STARTED);
		// TODO: Implement FrameStarted callback handling

		return false;
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

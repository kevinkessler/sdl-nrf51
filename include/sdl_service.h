/*
 * sdl_service_init.h
 *
 *  Created on: Oct 21, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#ifndef SDL_SERVICE_INIT_H_
#define SDL_SERVICE_INIT_H_

#include <stdint.h>
#include "ble_srv_common.h"
#include "sdl_config.h"

#define SDL_BASE_UUID		{{0xED,0x1F,0x5A,0x6F,0x7B,0xBE,0x50,0x80,0x27,0x45,0x5B,0xEE,0x00,0x00,0x00,0x00}}
#define SDL_SERVICE_UUID	0x0001
#define SDL_POWER_VALUE_1_CHAR_UUID 0x0100
#define SDL_POWER_VALUE_2_CHAR_UUID 0x0101
#define SDL_POWER_VALUE_3_CHAR_UUID 0x0102
#define SDL_BUTTON_VALUE_1_CHAR_UUID 0x0200
#define SDL_BUTTON_VALUE_2_CHAR_UUID 0x0201
#define SDL_BUTTON_VALUE_3_CHAR_UUID 0x0202
#define SDL_SWITCH_VALUE_1_CHAR_UUID 0x0300
#define SDL_SWITCH_VALUE_2_CHAR_UUID 0x0301
#define SDL_SWITCH_VALUE_3_CHAR_UUID 0x0302
#define SDL_IP_VALUE_CHAR_UUID 0x0400


typedef struct {
	uint16_t service_handle;
	ble_uuid_t uuid;
	ble_gatts_char_handles_t power_1_handle;
	ble_gatts_char_handles_t button_1_handle;
	ble_gatts_char_handles_t switch_1_handle;
	ble_gatts_char_handles_t powerip_1_handle;
	uint16_t conn_handle;
}ble_sdl_service_t;

void sdl_service_init(ble_sdl_service_t *sdl_service);
void sdl_service_handle_write(ble_evt_t *evt, ble_sdl_service_t *sdl_service, sdl_config_t *device_config);

#endif /* SDL_SERVICE_INIT_H_ */

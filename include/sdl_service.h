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
#define SDL_VALUE_SELECT_CHAR_UUID 0x0100

typedef struct {
	uint16_t service_handle;
	ble_uuid_t uuid;
	ble_gatts_char_handles_t char_handles;
	uint16_t conn_handle;
}ble_sdl_service_t;

void sdl_service_init(ble_sdl_service_t *sdl_service);
void sdl_service_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);

#endif /* SDL_SERVICE_INIT_H_ */

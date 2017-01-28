/*
 * sdl.h
 *
 *  Created on: Oct 22, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#ifndef INCLUDE_SDL_CONFIG_H_
#define INCLUDE_SDL_CONFIG_H_

#include <stdint.h>
#include "pstorage.h"

#define BLE_FLASH_PAGE_END ((NRF_UICR->BOOTLOADERADDR != 0xFFFFFFFF) ? \
                           ((NRF_UICR->BOOTLOADERADDR / ((uint16_t)NRF_FICR->CODEPAGESIZE))-1) : (NRF_FICR->CODESIZE -1))
#define DEVICE_TYPE_0 0
#define DEVICE_TYPE_1 1
#define DEVICE_TYPE_2 2

#define DEVICE_SWITCH 0
#define DEVICE_POWER 1
#define DEVICE_POWER_WITH_IP 2
#define DEVICE_BUTTON 3
#define DEVICE_POWER_WITH_BUTTON 4

typedef struct {
	uint16_t power_value_1;
	uint16_t power_value_2;
	uint16_t switch_value_1;
	uint16_t switch_value_2;
	uint16_t button_value_1;
	uint16_t button_value_2;
	uint16_t powerip_value;
}mesh_values_t;

typedef struct {
	pstorage_handle_t pstorage_handle;
	mesh_values_t values;
	uint8_t device_type;
}sdl_config_t;

void read_device_configuration(sdl_config_t *config);
void flash_cb_handler(pstorage_handle_t  *handle, uint8_t op_code, uint32_t result, uint8_t *p_data,
		              uint32_t data_len);
void write_value_handle(sdl_config_t *config);


#endif /* INCLUDE_SDL_CONFIG_H_ */

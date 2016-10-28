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

#define BLE_FLASH_PAGE_END ((NRF_UICR->BOOTLOADERADDR != 0xFFFFFFFF) ? \
                           (NRF_UICR->BOOTLOADERADDR / ((uint16_t)NRF_FICR->CODEPAGESIZE)) : (NRF_FICR->CODESIZE -1))
#define DEVICE_TYPE_0 0
#define DEVICE_TYPE_1 1
#define DEVICE_TYPE_2 2

#define DEVICE_SWITCH 0
#define DEVICE_POWER 1
#define DEVICE_POWER_WITH_IP 2
#define DEVICE_BUTTON 3

typedef struct {
	uint16_t value_handle;
	uint8_t device_type;
}sdl_config_t;

void read_device_configuration(sdl_config_t *config);
void flash_cb_handler(uint32_t sd_evt);
void write_value_handle(sdl_config_t *config);


#endif /* INCLUDE_SDL_CONFIG_H_ */

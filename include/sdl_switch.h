/*
 * sdl_switch.h
 *
 *  Created on: Oct 27, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#ifndef INCLUDE_SDL_SWITCH_H_
#define INCLUDE_SDL_SWITCH_H_

#include "sdl_service.h"

#define SWITCH_PIN	16
#define APP_TIMER_PRESCALER 0

void switch_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);
void sdl_switch_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);

#endif /* INCLUDE_SDL_SWITCH_H_ */

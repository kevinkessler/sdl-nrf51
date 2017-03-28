/*
 * sdl_button.h
 *
 *  Created on: Jan 26, 2017
 *      Author: kevin
 */

#ifndef INCLUDE_SDL_BUTTON_H_
#define INCLUDE_SDL_BUTTON_H_

#include "sdl_service.h"

#define BUTTON_PIN	15
#define APP_TIMER_PRESCALER 0

void button_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);
void sdl_button_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);

#endif /* INCLUDE_SDL_BUTTON_H_ */

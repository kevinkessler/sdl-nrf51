/*
 * sdl_power.h
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#ifndef INCLUDE_SDL_POWER_H_
#define INCLUDE_SDL_POWER_H_

#include "sdl_config.h"
#include "sdl_service.h"
#include "rbc_mesh.h"

#define RELAY_PIN 10

void power_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);
void power_update_val(rbc_mesh_event_t* p_evt,sdl_config_t *device_config);
void sdl_power_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);

#endif /* INCLUDE_SDL_POWER_H_ */

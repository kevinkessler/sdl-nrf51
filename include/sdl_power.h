/*
 * sdl_power.h
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#ifndef INCLUDE_SDL_POWER_H_
#define INCLUDE_SDL_POWER_H_

#include "sdl_config.h"
#include "rbc_mesh.h"

#define RELAY_PIN 10

void power_init();
void power_update_val(rbc_mesh_event_t* p_evt,sdl_config_t *device_config);

#endif /* INCLUDE_SDL_POWER_H_ */

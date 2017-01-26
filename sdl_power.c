/*
 * sdl_power.c
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#include <stdint.h>
#include <stdio.h>
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "rbc_mesh.h"
#include "include/sdl_power.h"
#include "SEGGER_RTT.h"

void power_init()
{
    nrf_gpio_cfg_output(RELAY_PIN);

}

void power_update_val(rbc_mesh_event_t* p_evt,sdl_config_t *device_config)
{
	SEGGER_RTT_WriteString(0,"Mesh Value...\n");
	if(p_evt->params.rx.value_handle == device_config->value_handle ) {
		if(p_evt->params.rx.p_data[0] == 0) {
			nrf_gpio_pin_clear(RELAY_PIN);
		}
		else {

			nrf_gpio_pin_set(RELAY_PIN);
		}
	}
}


/*
 * sdl_switch.c
 *
 *  Created on: Oct 27, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#include <stdint.h>
#include <stdio.h>
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"
#include "rbc_mesh.h"
#include "include/sdl_config.h"
#include "include/sdl_switch.h"

static app_timer_id_t delay_timer_id;
extern sdl_config_t device_config;

static void button_timer_handler(void *p_context)
{

	uint8_t mesh_data[1];

	if (nrf_drv_gpiote_in_is_set(SWITCH_PIN))
	{
		mesh_data[0]=1;

	}
	else
	{
		mesh_data[0]=0;
	}

	rbc_mesh_value_set(device_config.value_handle,mesh_data,1);
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	app_timer_stop(delay_timer_id);
	app_timer_start(delay_timer_id,APP_TIMER_TICKS(50, APP_TIMER_PRESCALER),NULL);
}

void switch_init(void)
{
    uint32_t err_code;


    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    config.pull=NRF_GPIO_PIN_PULLDOWN;
    err_code = nrf_drv_gpiote_in_init(SWITCH_PIN, &config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_toggle(19);

    err_code = app_timer_create(&delay_timer_id,
                                    APP_TIMER_MODE_SINGLE_SHOT,
                                    button_timer_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(SWITCH_PIN,true);
    APP_ERROR_CHECK(err_code);
}


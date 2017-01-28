/*
 * sdl_button.c
 *
 *  Created on: Jan 26, 2017
 *      Author: kevin
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"
#include "rbc_mesh.h"
#include "app_scheduler.h"
#include "include/sdl_config.h"
#include "include/sdl_service.h"
#include "include/sdl_button.h"
#include "SEGGER_RTT.h"

//static app_timer_id_t button_delay_timer_id;
extern sdl_config_t device_config;

static void button_timer_handler(void *p_event_data, uint16_t event_size)
{
//	SEGGER_RTT_WriteString(0,"Timer...\n");

	uint8_t mesh_data[23];
	uint16_t length;

	rbc_mesh_value_get(device_config.values.button_value_1,mesh_data,&length);

	SEGGER_RTT_printf(0,"Value Handle %x Mesh Value %x \n", device_config.values.button_value_1,mesh_data[0]);
	mesh_data[0] = (mesh_data[0]==0) ? 1 : 0;

	rbc_mesh_value_set(device_config.values.button_value_1,mesh_data,1);
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//	SEGGER_RTT_WriteString(0,"Button Event...\n");
	app_sched_event_put(NULL, 0, button_timer_handler);
//	app_timer_stop(button_delay_timer_id);
//	app_timer_start(button_delay_timer_id,APP_TIMER_TICKS(50, APP_TIMER_PRESCALER),NULL);
}

void button_init(ble_sdl_service_t *sdl_service, sdl_config_t *config)
{
    uint32_t err_code;

    rbc_mesh_value_enable(config->values.button_value_1);
    rbc_mesh_persistence_set(config->values.button_value_1, true);
    rbc_mesh_tx_event_set(config->values.button_value_1, true);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t pin_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    pin_config.pull=NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &pin_config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);

/*    err_code = app_timer_create(&button_delay_timer_id,
                                    APP_TIMER_MODE_SINGLE_SHOT,
                                    button_timer_handler); */
//    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_PIN,true);
    APP_ERROR_CHECK(err_code);

    sdl_button_char_init(sdl_service, config);
}

void sdl_button_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config)
{
	uint32_t err_code;

	ble_uuid_t char_uuid;
	ble_uuid128_t base_uuid = SDL_BASE_UUID;

	char_uuid.uuid = SDL_BUTTON_VALUE_1_CHAR_UUID;
	err_code = sd_ble_uuid_vs_add(&base_uuid,&char_uuid.type);

	ble_gatts_attr_md_t attr_md;
	memset(&attr_md, 0, sizeof(attr_md));
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	ble_gatts_attr_t    attr_char_value;
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;
	attr_char_value.max_len = 2;
	attr_char_value.init_len = 2;
	uint16_t value=config->values.button_value_1;
	attr_char_value.p_value=(uint8_t *)&value;

	ble_gatts_char_md_t char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;

	err_code = sd_ble_gatts_characteristic_add(sdl_service->service_handle,
	                                   &char_md,
	                                   &attr_char_value,
	                                   &sdl_service->button_1_handle);

	APP_ERROR_CHECK(err_code);
}


/*
 * sdl_power.c
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "rbc_mesh.h"
#include "ble_srv_common.h"
#include "include/sdl_power.h"
#include "include/sdl_service.h"


void power_init(ble_sdl_service_t *sdl_service, sdl_config_t *config)
{
    nrf_gpio_cfg_output(RELAY_PIN);

    uint8_t mesh_data[1];
    mesh_data[0]=0;
    rbc_mesh_value_set(config->values.power_value_1,mesh_data,1);

    rbc_mesh_value_enable(config->values.power_value_1);
    rbc_mesh_persistence_set(config->values.power_value_1, true);
    sdl_power_char_init(sdl_service, config);
}

void sdl_power_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config)
{
	uint32_t err_code;

	ble_uuid_t char_uuid;
	ble_uuid128_t base_uuid = SDL_BASE_UUID;

	char_uuid.uuid = SDL_POWER_VALUE_1_CHAR_UUID;
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
	uint16_t value=config->values.power_value_1;
	attr_char_value.p_value=(uint8_t *)&value;

	ble_gatts_char_md_t char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;

	err_code = sd_ble_gatts_characteristic_add(sdl_service->service_handle,
	                                   &char_md,
	                                   &attr_char_value,
	                                   &sdl_service->power_1_handle);

	APP_ERROR_CHECK(err_code);
}

void power_update_val(rbc_mesh_event_t* p_evt,sdl_config_t *device_config)
{
	if(p_evt->params.rx.value_handle == device_config->values.power_value_1 ) {
		if(p_evt->params.rx.p_data[0] == 0) {
			nrf_gpio_pin_clear(RELAY_PIN);
		}
		else {

			nrf_gpio_pin_set(RELAY_PIN);
		}
	}
}


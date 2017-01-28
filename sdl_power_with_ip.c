/*
 * sdl_power_with_ip.c
 *
 *  Created on: Oct 27, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_error.h"
#include "app_uart.h"
#include "rbc_mesh.h"
#include "include/sdl_power_with_ip.h"
#include "include/sdl_service.h"

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_CHECK(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_CHECK(p_event->data.error_code);
    }
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
    	uint8_t dummy;
    	app_uart_get(&dummy);
    }
}

void power_with_ip_init(ble_sdl_service_t *sdl_service, sdl_config_t *config)
{
	uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          0,
          0,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_INIT(&comm_params,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);

    sdl_powerip_char_init(sdl_service, config);
}

void power_with_ip_update_val(rbc_mesh_event_t* p_evt,sdl_config_t *device_config)
{
	if(p_evt->params.rx.value_handle == device_config->values.powerip_value ) {
		if(p_evt->params.rx.p_data[0] == 0) {
			app_uart_put('F');
		}
		else {
			app_uart_put('N');
		}
	}
}

void sdl_powerip_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config)
{
	uint32_t err_code;

	ble_uuid_t char_uuid;
	ble_uuid128_t base_uuid = SDL_BASE_UUID;

	char_uuid.uuid = SDL_IP_VALUE_CHAR_UUID;
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
	uint16_t value=config->values.powerip_value;
	attr_char_value.p_value=(uint8_t *)&value;

	ble_gatts_char_md_t char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;

	err_code = sd_ble_gatts_characteristic_add(sdl_service->service_handle,
	                                   &char_md,
	                                   &attr_char_value,
	                                   &sdl_service->powerip_1_handle);

	APP_ERROR_CHECK(err_code);
}

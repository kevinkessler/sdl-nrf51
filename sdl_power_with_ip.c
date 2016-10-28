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
#include "app_error.h"
#include "app_uart.h"
#include "rbc_mesh.h"
#include "include/sdl_power_with_ip.h"

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

void power_with_ip_init()
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
}

void power_with_ip_update_val(rbc_mesh_event_t* p_evt,sdl_config_t *device_config)
{
	if(p_evt->params.rx.value_handle == device_config->value_handle ) {
		if(p_evt->params.rx.p_data[0] == 0) {
			app_uart_put('F');
		}
		else {
			app_uart_put('N');
		}
	}
}

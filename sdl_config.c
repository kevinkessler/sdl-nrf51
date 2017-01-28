/*
 * sdl.c
 *
 *  Created on: Oct 22, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#include "include/sdl_config.h"
#include <string.h>
#include <stdint.h>
#include "app_error.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "nrf_gpio.h"
#include "pstorage.h"

#include "SEGGER_RTT.h"

#define BLOCK_SIZE 16

/* Write data must be available even after the function exits, so it can't be on the stack*/
static uint8_t data_buffer[4];
volatile uint8_t pstorage_ready = 0;
static pstorage_block_t pstorage_wait_handle = 0;

void flash_cb_handler(pstorage_handle_t *handle, uint8_t op_code, uint32_t result, uint8_t *p_data,
		              uint32_t data_len)
{
	SEGGER_RTT_printf(0,"Flash Opcode %d, Result %d\n",op_code, result);

	switch(op_code)
	{
		case PSTORAGE_CLEAR_OP_CODE:
			if(pstorage_wait_handle == handle->block_id)
				pstorage_ready=0;
			break;
		default:
			break;

	}
}

void read_device_configuration(sdl_config_t *config)
{
	uint32_t err_code;
	pstorage_module_param_t param;
	pstorage_handle_t block_handle;

	param.block_count=1;
	param.block_size=BLOCK_SIZE;
	param.cb=flash_cb_handler;

	err_code=pstorage_register(&param,&(config->pstorage_handle));
	APP_ERROR_CHECK(err_code);

	err_code=pstorage_block_identifier_get(&(config->pstorage_handle),0,&block_handle);
	APP_ERROR_CHECK(err_code);

	err_code=pstorage_load(data_buffer,&block_handle,sizeof(mesh_values_t),0);
	APP_ERROR_CHECK(err_code);

	memcpy((void *)&config->values,data_buffer,sizeof(mesh_values_t));
	//config->values=*((mesh_values_t *)data_buffer);
	SEGGER_RTT_printf(0,"Listen Value = %x, Addr = %x\n",config->values.power_value_1,block_handle.block_id);

	nrf_gpio_cfg_input(DEVICE_TYPE_0,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(DEVICE_TYPE_1,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(DEVICE_TYPE_2,NRF_GPIO_PIN_PULLDOWN);

	config->device_type=nrf_gpio_pin_read(DEVICE_TYPE_0)|nrf_gpio_pin_read(DEVICE_TYPE_1)<<1|nrf_gpio_pin_read(DEVICE_TYPE_2)<<2;
	SEGGER_RTT_printf(0,"Device Type = %x\n",config->device_type);

}

void write_value_handle(sdl_config_t *config)
{
	uint32_t err_code;
	pstorage_handle_t block_handle;

	err_code=pstorage_block_identifier_get(&(config->pstorage_handle),0,&block_handle);
	APP_ERROR_CHECK(err_code);

	uint8_t *b =(uint8_t *)(&config->values);

	pstorage_wait_handle=block_handle.block_id;
	pstorage_ready=1;

	SEGGER_RTT_printf(0,"Block ID %x\n", block_handle.block_id);

	err_code=pstorage_clear(&block_handle,BLOCK_SIZE);
	APP_ERROR_CHECK(err_code);
	while(pstorage_ready)
	{
		sd_app_evt_wait();
		app_sched_execute();
	}
	err_code=pstorage_store(&block_handle, b,sizeof(mesh_values_t),0);
	APP_ERROR_CHECK(err_code);

}



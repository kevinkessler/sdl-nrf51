/*
 * sdl.c
 *
 *  Created on: Oct 22, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#include "include/sdl_config.h"

#include <stdint.h>
#include "app_error.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"


volatile uint8_t flash_wait=0;

void flash_cb_handler(uint32_t sd_evt)
{
	SEGGER_RTT_printf(0,"SD Evt %d\n",sd_evt == NRF_EVT_FLASH_OPERATION_SUCCESS ? 1 : 0 );
	flash_wait=0;
}



void read_device_configuration(sdl_config_t *config)
{

	uint32_t *addr;
	addr = (uint32_t *)(BLE_FLASH_PAGE_END * NRF_FICR->CODEPAGESIZE);
	config->value_handle=(uint16_t) *addr;
	SEGGER_RTT_printf(0,"Listen Value = %x\n",config->value_handle);

	nrf_gpio_cfg_input(DEVICE_TYPE_0,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(DEVICE_TYPE_1,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(DEVICE_TYPE_2,NRF_GPIO_PIN_PULLDOWN);

	config->device_type=nrf_gpio_pin_read(DEVICE_TYPE_0)|nrf_gpio_pin_read(DEVICE_TYPE_1)<<1|nrf_gpio_pin_read(DEVICE_TYPE_2)<<2;
	SEGGER_RTT_printf(0,"Device Type = %x\n",config->device_type);
}

void write_value_handle(sdl_config_t *config)
{
	uint32_t err_code;

	flash_wait = 1;
	err_code = sd_flash_page_erase(BLE_FLASH_PAGE_END);
	APP_ERROR_CHECK(err_code);
	SEGGER_RTT_printf(0,"sd_flash %x\n",err_code);
	while(flash_wait)
	{
		sd_app_evt_wait();
		app_sched_execute();

	}


	flash_wait = 1;
	uint32_t new_value= config->value_handle;
	SEGGER_RTT_printf(0,"After Wait %x\n",new_value);

	uint32_t *addr;
	addr = (uint32_t *)(BLE_FLASH_PAGE_END * NRF_FICR->CODEPAGESIZE);

	err_code = sd_flash_write(addr,&new_value,1);
	APP_ERROR_CHECK(err_code);
	SEGGER_RTT_printf(0,"sd_write_flash %x\n",err_code);
	while(flash_wait)
	{
		sd_app_evt_wait();
		app_sched_execute();
	}


	for (int n=0;n<10;n++)
	{
		SEGGER_RTT_printf(0,"Addr %x = %x\n",addr+n,*(addr+n));
	}

}



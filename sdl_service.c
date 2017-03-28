/*
 * sdl_service.c
 *
 *  Created on: Oct 21, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#include <string.h>

#include "include/sdl_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "rbc_mesh.h"
#include "SEGGER_RTT.h"

void sdl_service_handle_write(ble_evt_t *evt, ble_sdl_service_t *sdl_service, sdl_config_t *device_config)
{
	uint16_t value = evt->evt.gatts_evt.params.write.data[1] << 8 | evt->evt.gatts_evt.params.write.data[0];
	SEGGER_RTT_printf(0,"GATTS Write Handle %x Data %x Len %x\n",evt->evt.gatts_evt.params.write.handle,
				value,evt->evt.gatts_evt.params.write.len);

	if(evt->evt.gatts_evt.params.write.handle == sdl_service->button_1_handle.value_handle)
	{
	    rbc_mesh_persistence_set(device_config->values.button_value_1, false);
	    rbc_mesh_tx_event_set(device_config->values.button_value_1, false);
		device_config->values.button_value_1=value;
		write_value_handle(device_config);
	    rbc_mesh_value_enable(device_config->values.button_value_1);
	    rbc_mesh_persistence_set(device_config->values.button_value_1, true);
	    rbc_mesh_tx_event_set(device_config->values.button_value_1, true);
	}
	else if(evt->evt.gatts_evt.params.write.handle == sdl_service->switch_1_handle.value_handle)
	{
		device_config->values.switch_value_1=value;
		write_value_handle(device_config);
	}
	else if(evt->evt.gatts_evt.params.write.handle == sdl_service->power_1_handle.value_handle)
	{
		device_config->values.power_value_1=value;
		write_value_handle(device_config);
	}
	else if(evt->evt.gatts_evt.params.write.handle == sdl_service->powerip_1_handle.value_handle)
	{
		device_config->values.powerip_value=value;
		write_value_handle(device_config);
	}
}

void sdl_service_init(ble_sdl_service_t *sdl_service)
{
	uint32_t err_code;

	ble_uuid_t service_uuid;
	ble_uuid128_t base_uuid = SDL_BASE_UUID;

	service_uuid.uuid=SDL_SERVICE_UUID;
	err_code = sd_ble_uuid_vs_add(&base_uuid,&service_uuid.type);
	sdl_service->uuid = service_uuid;
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &sdl_service->service_handle);
	APP_ERROR_CHECK(err_code);

	sdl_service->conn_handle = BLE_CONN_HANDLE_INVALID;

}

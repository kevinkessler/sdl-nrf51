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
#include "SEGGER_RTT.h"

void sdl_service_char_init(ble_sdl_service_t *sdl_service)
{
	uint32_t err_code;

	ble_uuid_t char_uuid;
	ble_uuid128_t base_uuid = SDL_BASE_UUID;

	char_uuid.uuid = SDL_VALUE_SELECT_CHAR_UUID;
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
	uint16_t value=0x1234;
	attr_char_value.p_value=(uint8_t *)&value;

	ble_gatts_char_md_t char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;

	err_code = sd_ble_gatts_characteristic_add(sdl_service->service_handle,
	                                   &char_md,
	                                   &attr_char_value,
	                                   &sdl_service->char_handles);

	APP_ERROR_CHECK(err_code);
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

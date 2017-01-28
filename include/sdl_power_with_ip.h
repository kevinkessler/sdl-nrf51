/*
 * sdk_power_with_ip.h
 *
 *  Created on: Oct 27, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */

#ifndef INCLUDE_SDL_POWER_WITH_IP_H_
#define INCLUDE_SDL_POWER_WITH_IP_H_

#include "sdl_config.h"
#include "rbc_mesh.h"
#include "sdl_service.h"

// Power With Ip defines
#define RX_PIN_NUMBER 11
#define TX_PIN_NUMBER 9
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

void power_with_ip_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);
void power_with_ip_update_val(rbc_mesh_event_t* p_evt, sdl_config_t *device_config);
void sdl_powerip_char_init(ble_sdl_service_t *sdl_service, sdl_config_t *config);

#endif /* INCLUDE_SDL_POWER_WITH_IP_H_ */

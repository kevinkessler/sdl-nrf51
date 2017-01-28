/*
 * main.c
 *
 *  Created on: Oct 21, 2016
 *      Author: Kevin
 *
 *  Copyright (c) 2016 Kevin Kessler - All Rights Reserved
 */


#include "rbc_mesh.h"
#include "mesh_aci.h"
#include "softdevice_handler_appsh.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble_advdata.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nrf_delay.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "device_manager.h"
#include "pstorage.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "include/sdl_config.h"
#include "include/sdl_service.h"
#include "include/sdl_power_with_ip.h"
#include "include/sdl_switch.h"
#include "include/sdl_power.h"
#include "include/sdl_button.h"

#include "SEGGER_RTT.h"

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(100, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(200, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */
#define SDL_ADV_INTERVAL                 320                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 200 ms). */
//#define APP_ADV_TIMEOUT_IN_SECONDS       600                                        /**< The advertising timeout in units of seconds. */

#define MESH_ACCESS_ADDR        (0xA541A68F)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SRC          (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)

#define APP_TIMER_PRESCALER             0                                            /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4					                         /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         40                                            /**< Size of timer operation queues. */


#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE,BLE_STACK_HANDLER_SCHED_EVT_SIZE)  /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10

//app_timer_id_t delay_timer_id;
ble_sdl_service_t sdl_service;
sdl_config_t device_config;
ble_gap_adv_params_t ble_adv_params;

static ble_dfu_t m_dfus;
static dm_application_instance_t dm_handle;

// Forward Declarations
void sd_ble_evt_handler(ble_evt_t* p_ble_evt);
void sys_evt_handler(uint32_t sd_evt);
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,dm_event_t const  * p_event, ret_code_t event_result);
static void app_context_load(dm_handle_t const * p_handle);

/** @brief General error handler. */
static inline void error_loop(void)
{
    __disable_irq();
    while (true)
    {
        __WFE();
    }
}

/**
* @brief Softdevice crash handler, never returns
*
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
	SEGGER_RTT_printf(0,"Assert %x, line %d, file %s\n",pc,line_num,p_file_name);
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
*
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	SEGGER_RTT_printf(0,"Error %x, line %d, file %s\n",error_code,line_num,p_file_name);
    error_loop();
}

/** @brief Hardware fault handler. */
void HardFault_Handler(void)
{
	SEGGER_RTT_WriteString(0,"Hardfault");
    error_loop();
}

static void reset_prepare(void)
{
    uint32_t err_code;
    if (sdl_service.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(sdl_service.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);

    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        err_code = sd_ble_gap_adv_stop();
        APP_ERROR_CHECK(err_code);
    }

    nrf_delay_ms(500);
}

/* ------------------------------------------Initialization Routines------------------------------------------------------------ */
static void timers_init(void)
{
	    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}

static void ble_stack_init(void)
{
    uint32_t err_code;

    SOFTDEVICE_HANDLER_APPSH_INIT(MESH_CLOCK_SRC, true);


    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    ble_enable_params.gatts_enable_params.service_changed = 1;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    err_code=softdevice_ble_evt_handler_set(sd_ble_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code=softdevice_sys_evt_handler_set(sys_evt_handler);
    APP_ERROR_CHECK(err_code);

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void gap_params_init(void)
{
	uint32_t err_code;

    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    ble_gap_addr_t my_addr;

    err_code = sd_ble_gap_address_get(&my_addr);
    APP_ERROR_CHECK(err_code);

    char name[64];
    sprintf(name, "sdl#%d",
        ((uint16_t) my_addr.addr[4] << 8) | (my_addr.addr[5]));

    err_code = sd_ble_gap_device_name_set(&sec_mode, (uint8_t*) name, strlen(name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


static void mesh_init(void)
{
    rbc_mesh_init_params_t init_params;
    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SRC;
    init_params.tx_power = RBC_MESH_TXPOWER_0dBm;

    uint32_t error_code;
    error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

}

static void hw_init(void)
{
	sdl_service_init(&sdl_service);

	if(device_config.device_type==DEVICE_SWITCH)
	{
		switch_init(&sdl_service,&device_config);
	}

	if((device_config.device_type==DEVICE_POWER)||
	   (device_config.device_type==DEVICE_POWER_WITH_BUTTON))
	{
		power_init(&sdl_service,&device_config);
	}

	if((device_config.device_type==DEVICE_BUTTON)||
	   (device_config.device_type==DEVICE_POWER_WITH_BUTTON))
	{
		button_init(&sdl_service,&device_config);
	}

	if(device_config.device_type==DEVICE_POWER_WITH_IP)
	{
		power_with_ip_init(&sdl_service,&device_config);
	}


}

static void dfu_init(void)
{
	uint32_t err_code;



	// Initialize DFU Services
	ble_dfu_init_t   dfus_init;

	memset(&dfus_init, 0, sizeof(dfus_init));
	dfus_init.evt_handler  = dfu_app_on_dfu_evt;
	dfus_init.error_handler = NULL;
	dfus_init.evt_handler = dfu_app_on_dfu_evt;
	dfus_init.revision = 0x0001;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(dm_handle);
}


static void advertising_init(void)
{
    uint32_t err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED | BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_advdata_t srdata;
    memset(&srdata,0,sizeof(srdata));
    ble_uuid_t adv_uuids[] = {{sdl_service.uuid.uuid,sdl_service.uuid.type}};
    srdata.uuids_more_available.uuid_cnt = sizeof(adv_uuids)/sizeof(adv_uuids[0]);
    srdata .uuids_more_available.p_uuids = adv_uuids;

    err_code=ble_advdata_set(&advdata,&srdata);
    APP_ERROR_CHECK(err_code);

    memset(&ble_adv_params,0,sizeof(ble_adv_params));
    ble_adv_params.type=BLE_GAP_ADV_TYPE_ADV_IND;
    ble_adv_params.interval=SDL_ADV_INTERVAL;
    ble_adv_params.timeout = 0;

    err_code=sd_ble_gap_adv_start(&ble_adv_params);
    APP_ERROR_CHECK(err_code);

}

static void device_manager_init()
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = 0};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);


    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond = 1;						/**< Perform bonding. */
    register_param.sec_param.mitm = 0;						/**< Man In The Middle protection not required. */
    register_param.sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;/**< No I/O capabilities. */
    register_param.sec_param.oob = 0;						/**< Out Of Band data not available. */
    register_param.sec_param.min_key_size = 7;				/**< Minimum encryption key size. */
    register_param.sec_param.max_key_size = 16;				/**< Maximum encryption key size. */

    register_param.evt_handler = device_manager_evt_handler;
    register_param.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&dm_handle, &register_param);
    APP_ERROR_CHECK(err_code);

}

/* ------------------------------------------Event Handlers------------------------------------------------------------ */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }

    return NRF_SUCCESS;
}

static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
        	/** 0x000C Handle of first application specific service when when service changed characteristic is present. */
        	/**0xFFFF Max handle value in BLE. */
            err_code = sd_ble_gatts_service_changed(dm_handle, 0x000C, 0xFFFF);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void ble_gatts_event_handler(ble_evt_t* evt)
{

    switch (evt->header.evt_id)
    {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        sd_ble_gatts_sys_attr_set(evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
        break;

    case BLE_GATTS_EVT_WRITE:
    	if(evt->evt.gatts_evt.conn_handle==sdl_service.conn_handle)
    		sdl_service_handle_write(evt, &sdl_service, &device_config);

        break;

    default:
        break;
    }
}

static void ble_gap_event_handler(ble_evt_t* evt)
{
//	uint32_t err_code;

    switch (evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
    	sdl_service.conn_handle = evt->evt.gap_evt.conn_handle;
        break;

    case BLE_GAP_EVT_DISCONNECTED:
    	sdl_service.conn_handle = BLE_CONN_HANDLE_INVALID;
        sd_ble_gap_adv_start(&ble_adv_params);
        break;

//    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//        err_code = sd_ble_gap_sec_params_reply(sdl_service.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
//        APP_ERROR_CHECK(err_code);
//          break;

    case BLE_GAP_EVT_CONN_SEC_UPDATE:
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        break;

    default:
        break;
    }
}


/**
* @brief Mesh framework event handler.
*
* @param[in] p_evt Mesh event propagated from framework.
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{

	SEGGER_RTT_printf(0,"Mesh Event %x\n",p_evt->type);
    switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        case RBC_MESH_EVENT_TYPE_TX:
        	if(device_config.device_type==DEVICE_POWER_WITH_IP)
        	{
        		power_with_ip_update_val(p_evt,&device_config);
        	}
        	else if(device_config.device_type==DEVICE_POWER)
        	{
        		power_update_val(p_evt, &device_config);
        	}
        	else if(device_config.device_type==DEVICE_POWER_WITH_BUTTON)
        	{
        		power_update_val(p_evt, &device_config);
        	}
        	break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
        case RBC_MESH_EVENT_TYPE_DFU_NEW_FW_AVAILABLE:
        	// Do some DFU Stuff
        case RBC_MESH_EVENT_TYPE_DFU_RELAY_REQ:
        case RBC_MESH_EVENT_TYPE_DFU_SOURCE_REQ:
        case RBC_MESH_EVENT_TYPE_DFU_START:
        case RBC_MESH_EVENT_TYPE_DFU_END:
        case RBC_MESH_EVENT_TYPE_DFU_BANK_AVAILABLE:
            break;
    }
}

void nrf_adv_conn_evt_handler(ble_evt_t* evt)
{
    switch (evt->header.evt_id & 0xF0)
    {
    case BLE_GAP_EVT_BASE:
        ble_gap_event_handler(evt);
        break;

    case BLE_GATTS_EVT_BASE:
        ble_gatts_event_handler(evt);
        break;

    default:
        break;
    }
}

void sd_ble_evt_handler(ble_evt_t* p_ble_evt)
{

    rbc_mesh_ble_evt_handler(p_ble_evt);
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    nrf_adv_conn_evt_handler(p_ble_evt);
    dm_ble_evt_handler(p_ble_evt);
}


void sys_evt_handler(uint32_t sd_evt)
{
	pstorage_sys_event_handler(sd_evt);
	rbc_mesh_sd_evt_handler(sd_evt);
}

int main(void)
{
	SEGGER_RTT_WriteString(0,"Starting...\n");

    timers_init();
	ble_stack_init();

	/* Device Manger must be initialized before read_device_config because pstorage is init'd in dm_init */
	device_manager_init();
	gap_params_init();
    read_device_configuration(&device_config);
	mesh_init();
	hw_init();
	dfu_init();
	advertising_init();
    rbc_mesh_event_t evt;

    while (true)
    {
    	app_sched_execute();

        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_event_release(&evt);
        }

        sd_app_evt_wait();
    }
}


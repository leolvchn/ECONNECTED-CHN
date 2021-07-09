/**
 * Copyright (c) 2013 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_ant_hrs_main main.c
 * @{
 * @ingroup ble_sdk_app_ant_hrs
 * @brief HRM sample application using both BLE and ANT.
 *
 * The application uses the BLE Heart Rate Service (and also the Device Information
 * services), and the ANT HRM RX profile.
 *
 * It will open a receive channel which will connect to an ANT HRM TX profile device when the
 * application starts. The received data will be propagated to a BLE central through the
 * BLE Heart Rate Service.
 *
 * The ANT HRM TX profile device simulator SDK application
 * (Board\pca10003\ant\ant_hrm\hrm_tx_buttons) can be used as a peer ANT device. By changing
 * ANT_HRMRX_NETWORK_KEY to the ANT+ Network Key, the application will instead be able to connect to
 * an ANT heart rate belt.
 *
 * @note The ANT+ Network Key is available for ANT+ Adopters. Please refer to
 *       http://thisisant.com to become an ANT+ Adopter and access the key.
 *
 * @note This application is based on the BLE Heart Rate Service Sample Application
 *       (Board\nrf6310\ble\ble_app_hrs). Please refer to this application for additional
 *       documentation.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_conn_state.h"
#include "ble_conn_params.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
//#include "ble_hrs.h"
#include "ble_dis.h"
#include "bsp.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_db_discovery.h"
#include "sensorsim.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ant_search_config.h"
#include "ble_nus_Domyos.h"
#include "app_uart.h"
#include "ble_ftms.h"
#include "ble_hrs.h"
#include "ble_hrs_c.h"

#include "nrf_ble_scan.h"

#include "fds.h"

#include "dkn_cmd.h"
#include "m_system.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "ant_error.h"
#include "ant_key_manager.h"
#include "ant_hrm.h"
#include "ant_parameters.h"
#include "ant_interface.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define WAKEUP_BUTTON_ID                0                                            /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID       1                                            /**< Button used for deleting all bonded centrals during startup. */

#define DEVICE_NAME                     "Domyos-TC-xxxx"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Domyos"                        /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM												"Domyos ANT"
#define FW_STR													"V1.0 210706"


/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT              (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)

#define APP_ADV_INTERVAL                160                                           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION                0                                        /**< The advertising duration in units of seconds. */

#define APP_BLE_CONN_CFG_TAG            1                                            /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                            /**< Whether or not to include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device */

#define SECOND_1_25_MS_UNITS            64                                          /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                          /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 2)                   /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS)                       /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ANT_HRMRX_ANT_CHANNEL           0                                            /**< Default ANT Channel. */
#define ANT_HRMRX_DEVICE_NUMBER         0                                            /**< Device Number. */
#define ANT_HRMRX_TRANS_TYPE            0                                            /**< Transmission Type. */
#define ANTPLUS_NETWORK_NUMBER          0                                            /**< Network number. */


#define ANT_500ms_DELL_INTERVAL     	APP_TIMER_TICKS(500) /**< Battery level measurement interval (ticks). */

#define UUID16_SIZE                 2



#define FILE_ID     0x1111
#define REC_KEY     0x2222

#define HART_RATE_SERVICE_UUID_IDX      0                                           /**< Hart Rate service UUID index in array. */

#define FLASH_CHECK 		0xAA51

static uint16_t           m_conn_handle_hrs_c                = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the HRS central application. */
//static volatile uint16_t  m_conn_handle_num_comp_central     = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the central that needs a numeric comparison button press. */
volatile uint16_t  m_conn_handle_peripheral  = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the peripheral that needs a numeric comparison button press. */

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint8_t                          m_adv_handle;                                /**< Advertising handle. */


MY_FLASH_DATA_DEF my_flash;

MY_FLASH_DATA_DEF my_flash_back;

fds_record_desc_t   record_desc = {0};
/* A record containing dummy configuration data. */
static fds_record_t const m_dummy_record =
{
    .file_id           = FILE_ID,
    .key               = REC_KEY,
    .data.p_data       = &my_flash,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(my_flash) + 3) / sizeof(uint32_t),
};


/**@brief Macro to unpack 16bit unsigned UUID from an octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

		
Ant_Hrm_Def Ant_Hrm;
Ble_Hr_Def  Ble_Hr;


/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t  * p_data;      /**< Pointer to data. */
    uint16_t   data_len;    /**< Length of data. */
} data_t;		

/********************************************************
*  BCD码                                   *
********************************************************/
static struct BCDdata{
	uint8_t _10000;
	uint8_t _1000;
	uint8_t _100;
	uint8_t _10;
	uint8_t _1;
}BCD;

//BLE_HRS_DEF(m_hrs);                                                                  /**< Heart rate service instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                      /**< Context for the Queued Write module.*/
NRF_BLE_GATT_DEF(m_gatt);                                                            /**< GATT module instance. */
BLE_FTMS_DEF(mftms, NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_HRS_DEF(m_hrs);
BLE_DB_DISCOVERY_DEF(m_db_disc);                                            /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                                   /**< Scanning Module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery, 2);                         /**< Database discovery module instances. */

ble_ftms_treadmill_t			treadmill;
ble_ftms_indoorbike_t			bike;
ble_ftms_cross_trainer_t	cross_trainer;
ble_ftms_rower_t					rower;

static ble_hrs_c_t m_hrs_c;                                         /**< Heart Rate Service client instance. */

ble_ftms_training_status_t	ftms_training_status;
ble_ftms_machine_status_t		ftms_machine_status;

System_Def	Sys;



typedef struct
{
    bool           is_connected;
    ble_gap_addr_t address;
} conn_peer_t;

//static conn_peer_t        m_connected_peers[NRF_BLE_LINK_COUNT];                         /**< Array of connected peers. */

/**@brief Names that the central application scans for, and that are advertised by the peripherals.
 *  If these are set to empty strings, the UUIDs defined below are used.
 */
//static const char m_target_periph_name[] = "";

//ant_hrm_profile_t m_ant_hrm;                                                         /**< ANT HRM profile instance. */
//HRM_DISP_CHANNEL_CONFIG_DEF(m_ant_hrm,
//                            ANT_HRMRX_ANT_CHANNEL,
//                            ANT_HRMRX_TRANS_TYPE,
//                            ANT_HRMRX_DEVICE_NUMBER,
//                            ANTPLUS_NETWORK_NUMBER,
//                            HRM_MSG_PERIOD_4Hz);
													
//NRF_SDH_ANT_OBSERVER(m_ant_hrm_observer, ANT_HRM_ANT_OBSERVER_PRIO, ant_hrm_disp_evt_handler, &m_ant_hrm);



APP_TIMER_DEF(m_timer_500ms_running_id);

APP_TIMER_DEF(m_timer_sys_reset_delay_id);

static void nus_data_handler(ble_nus_evt_t * p_evt);

static void mflash_init(void);
int8_t mflash_write(void);
static void scan_start(void);

extern void uart_init(void);
extern void uart_send_array(const uint8_t* buf, uint8_t len);

extern void dkn_cmd_dell_in_main_loop(void);
extern void uart_auto_hold(void);
extern void uart_conn_sta_changed(void);
extern void uart_has_hr_recv(uint8_t times);
extern void uart_send_cmd_after_nus_write(void);

static uint32_t he_uint32(uint8_t * data, uint8_t len)
{
    uint32_t data_he=0;
		
		do
		{
			len--;
			data_he <<= 8;
			data_he |= data[len];
		}while(len);
		
    return data_he;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
		NRF_LOG_INFO("sd_ble_gap_adv_start:%04X", err_code);
		
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
		
		Sys.status.ble_is_adving = true;
}


/**@brief Start receiving the ANT HRM data.
 */
//static void ant_hrm_rx_start(void)
//{
//    uint32_t err_code = ant_hrm_disp_open(&m_ant_hrm);
//	
//		NRF_LOG_INFO("ant_hrm_disp_open:%04X", err_code);
//    APP_ERROR_CHECK(err_code);
//}

static void ant_hrm_scan_start(void)
{
	
		uint32_t err_code = sd_ant_lib_config_set(ANT_LIB_CONFIG_MESG_OUT_INC_RSSI
                                            | ANT_LIB_CONFIG_MESG_OUT_INC_DEVICE_ID);
    APP_ERROR_CHECK(err_code);
	
		const ant_channel_config_t bs_channel_config =
    {
        .channel_number    = 0,
        .channel_type      = CHANNEL_TYPE_SLAVE,
        .ext_assign        = EXT_PARAM_ALWAYS_SEARCH,
        .rf_freq           = HRM_ANTPLUS_RF_FREQ,
        .transmission_type = ANT_HRMRX_TRANS_TYPE,
        .device_type       = HRM_DEVICE_TYPE,
        .device_number     = 0x00,              // Wild card
        .channel_period    = 0x00,              // This is not taken into account.
        .network_number    = ANTPLUS_NETWORK_NUMBER,
    };
	
		const ant_search_config_t bs_search_config =
    {
        .channel_number        = 0,
        .low_priority_timeout  = ANT_LOW_PRIORITY_TIMEOUT_DISABLE,
        .high_priority_timeout = ANT_HIGH_PRIORITY_SEARCH_DISABLE,
        .search_sharing_cycles = ANT_SEARCH_SHARING_CYCLES_DISABLE,
        .search_priority       = ANT_SEARCH_PRIORITY_2,
        .waveform              = ANT_WAVEFORM_FAST,
    };
		
		err_code = ant_channel_init(&bs_channel_config);
    APP_ERROR_CHECK(err_code);
		
		err_code = ant_search_init(&bs_search_config);
    APP_ERROR_CHECK(err_code);
		
		ANT_BUFFER_PTR readCfg;
		uint8_t buffer[10];
		readCfg.ucBufferSize = 10;
		readCfg.pucBuffer = &buffer[0];

		err_code = sd_ant_coex_config_get(0, &readCfg, NULL);
		readCfg.pucBuffer[0] = 0;
//		readCfg.pucBuffer[0] &= ~0x08; // disable fixed search hp interval priority config
//		readCfg.pucBuffer[3] *= 2;
		err_code = sd_ant_coex_config_set(0, &readCfg, NULL);
		APP_ERROR_CHECK(err_code);
		
//		sd_ant_search_channel_priority_set(0, 4);
		
		 err_code = sd_ant_channel_open(0);
		 NRF_LOG_INFO("sd_ant_channel_open:%04X", err_code);
		 APP_ERROR_CHECK(err_code);
		 Sys.status.ant_is_scaning = true;

		memset(&Ant_Hrm, 0, sizeof(Ant_Hrm));
}

/**@brief Attempt to both open the ant channel and start ble advertising.
*/
static void ant_and_adv_start(void)
{
    advertising_start();
    
	
		ant_hrm_scan_start();
}



static void timer_system_reset_delay_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	
	NVIC_SystemReset();
}




static void timer_500ms_running_handler(void * p_context)
{
	static uint8_t send_wait;
	static uint8_t timer_ftms_send;
	
	uint32_t err_code;
	int8_t rssi;
	uint8_t i,select, tx_buf[10], pulse;
	UNUSED_PARAMETER(p_context);
	
	send_wait++;
	
	if(++timer_ftms_send & 0x01)
	{
		if(my_flash.ftms.enable)
		{
			if(my_flash.ftms.type.b.treadmill_support)
			{
				ble_ftms_treadmill_send(&mftms, &treadmill);
			}
			if(my_flash.ftms.type.b.cross_trainer_support)
			{
				ble_ftms_cross_trainer_send(&mftms, &cross_trainer);
			}
			if(my_flash.ftms.type.b.rower_support)
			{
				ble_ftms_rower_send(&mftms, &rower);
			}
			if(my_flash.ftms.type.b.indoor_bike_support)
			{
				ble_ftms_indoorbike_send(&mftms, &bike);
			}
			
			ble_ftms_training_status_send(&mftms, &ftms_training_status);
//			ble_ftms_machine_status_send(&mftms, &ftms_machine_status);
		}
		
		if(Ant_Hrm.pulse)
			pulse = Ant_Hrm.pulse;
		else if(Ble_Hr.pulse)
			pulse = Ble_Hr.pulse;
		else
			pulse = dkn_get_pulse_from_uart();
		ble_hrs_heart_rate_measurement_send(&m_hrs, pulse);
	}
	
	
	if(Sys.ctrl.ant_scan_enable)
	{
		if(!Sys.status.ant_is_scaning)
		{
			err_code = sd_ant_channel_open(0);
			NRF_LOG_INFO("sd_ant_channel_open:%04X", err_code);
			APP_ERROR_CHECK(err_code);
			Sys.status.ant_is_scaning = true;
		}
		
		if(Ant_Hrm.status)
		{
			Ant_Hrm.lost_time++;
			if(Ant_Hrm.lost_time > 39) //0.5*40 = 20s
			{
				Ant_Hrm.lost_time = 0;
				if(Sys.status.ant_is_connected)
				{
					Sys.status.ant_is_connected = 0;
					uart_conn_sta_changed();
				}
				
				memset(&Ant_Hrm, 0, sizeof(Ant_Hrm));
				
				uart_has_hr_recv(3);
				
				Ant_Hrm.send_empty_times = 4;
				NRF_LOG_INFO("Ant Lost");
			}
		}
		else
		{
			if(Ant_Hrm.num)
			{
				Ant_Hrm.comfirm_time ++;
				
				if(Ant_Hrm.comfirm_time > 19) //0.5*20 = 10s
				{
					rssi = -127;
					for(i=0;i<Ant_Hrm.num;i++)
					{
						if(Ant_Hrm.device[i].rssi > rssi)
						{
							rssi = Ant_Hrm.device[i].rssi;
							select = i;
						}
					}
					
					Ant_Hrm.select_id = Ant_Hrm.device[select].id;
					
					Ant_Hrm.status = 1;
					if(Sys.status.ant_is_connected == 0)
					{
						Sys.status.ant_is_connected = 1;
						uart_conn_sta_changed();
					}
					NRF_LOG_INFO("comriem time over,Select:%d", Ant_Hrm.select_id);
				}
				
			}
			
		}
	}
	else
	{
		if(Sys.status.ant_is_scaning)
		{
			err_code = sd_ant_channel_close(0);
			NRF_LOG_INFO("sd_ant_channel_close:%04X", err_code);
			APP_ERROR_CHECK(err_code);
			Sys.status.ant_is_scaning = false;
			
			memset(&Ant_Hrm, 0, sizeof(Ant_Hrm));
		}
	}
	
	
	/*Ble_Hr.rssi = -127;
								Ble_Hr.pulse = 0;
								Ble_Hr.send_empty_times = 4;
								Ble_Hr.comfirm_time = 0;*/
	if(Sys.ctrl.ant_scan_enable)
	{
		if(Ble_Hr.rssi != -127)
		{
			Ble_Hr.comfirm_time++;
			if(Ble_Hr.comfirm_time > 19)
			{
				Ble_Hr.comfirm_time = 0;
				nrf_ble_scan_stop();
				Sys.status.ble_is_scaning = false;
				
				err_code = sd_ble_gap_connect(&Ble_Hr.addr,
                                          &m_scan.scan_params,
                                          &m_scan.conn_params,
                                          APP_BLE_CONN_CFG_TAG);
				Sys.status.ble_is_master_conn_ing = true;

				NRF_LOG_INFO("sd_ble_gap_connect(): 0x%x.\r\n", err_code);
				Ble_Hr.rssi = -127;
				Ble_Hr.conn_timeout = 0;
			}
		}
		else
		{
			if(++Ble_Hr.conn_timeout >= 10)
			{
				Ble_Hr.conn_timeout = 10;
				if(!Sys.status.ble_is_scaning && !Sys.status.ble_master_is_connected)
				{
					if(Sys.status.ble_is_master_conn_ing)
					{
						Sys.status.ble_is_master_conn_ing = false;
						err_code = sd_ble_gap_connect_cancel();
						NRF_LOG_INFO("sd_ble_gap_connect_cancel(): 0x%x", err_code);
					}
					scan_start();
				}
			}
		}
	}
	else
	{
		if(Sys.status.ble_is_scaning)
		{
			nrf_ble_scan_stop();
			NRF_LOG_INFO("nrf_ble_scan_stop");
			Sys.status.ble_is_scaning = false;
		}
		else if(Sys.status.ble_master_is_connected)
		{
			err_code = sd_ble_gap_disconnect(m_conn_handle_hrs_c, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			NRF_LOG_INFO("try disc master:%04X", err_code);
			Sys.status.ble_is_scaning = false;
		}
	}

	
	if(Sys.ctrl.ble_adv_enable)
	{
		if(!Sys.status.ble_is_adving && !Sys.status.ble_slave_is_connected)
		{
			advertising_start();
		}
	}
	else
	{
		if(Sys.status.ble_is_adving)
		{
			err_code = sd_ble_gap_adv_stop(m_adv_handle);
			NRF_LOG_INFO("sd_ble_gap_adv_stop:%04X", err_code);
			Sys.status.ble_is_adving = false;
		}
		else if(Sys.status.ble_slave_is_connected)
		{
			err_code = sd_ble_gap_disconnect(m_conn_handle_peripheral, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			NRF_LOG_INFO("sd_ble_gap_disconnect:%04X", err_code);
			Sys.status.ble_is_adving = false;
		}
	}
	

}

bool if_limit_min_max(int32_t value, int32_t min, int32_t max)
{
	if((value >= min)&&(value <= max))
		return true;
	
	return false;
}

void ftms_contol_point_data_handler(ble_ftms_t * p_ftms, ble_ftms_control_point_data_t* cp_data)
{
	ble_ftms_control_point_response_t	response;
	int32_t temp32;
	
	memset(&response, 0, sizeof(response));
//	NRF_LOG_INFO("ftms ctrl(op:%d,len:%d)\n", cp_data->op_code, cp_data->pram_len);
//	show_reg(cp_data->pram, cp_data->pram_len, 1);
	
	
	
	
	if(cp_data->op_code == CTRL_OP_Request_Control)	//0x00	Request Control
	{
		NRF_LOG_INFO("req ctrl\n");
		//my_uart_enable_auto_send();
//		Sys.ctrl.uart_dkn_cmd_enable = true;

		mftms.ctrl_point_enable = true;
		
		dkn_cmd.ctrl.b.is_ftms_app = 1;
		
		response.result = CTRL_RESULT_Success;
		
		response.op_code = cp_data->op_code;

		ble_ftms_control_point_send(p_ftms, &response);
		return ;

	}
	
	if(mftms.ctrl_point_enable)
	{
		switch(cp_data->op_code)
		{
			case CTRL_OP_Request_Reset: 										//0x01	Reset
				NRF_LOG_INFO("req rest\n");
			
				dkn_cmd.ctrl.b.req_reset = 1;
			
				response.result = CTRL_RESULT_Success;		
			break;
			case CTRL_OP_Set_Target_Speed: 								//0x02	Set Target Speed							--UINT16, in km/h with a resolution of 0.01 km/h
				temp32 = he_uint32(cp_data->pram, 2);
				if(if_limit_min_max(temp32, my_flash.ftms.range.speed.min, my_flash.ftms.range.speed.max))
				{
					NRF_LOG_INFO("set speed:%d.%02d km/h\n", temp32/100, temp32%100);

					temp32 /= 10;
					if(dkn_cmd.set.speed != temp32)
					{
						dkn_cmd.set.speed = temp32;
						dkn_cmd.ctrl.b.speed_changed = 1;
					}
					
					response.result = CTRL_RESULT_Success;
				}
				else
				{
					response.result = CTRL_RESULT_Invalid_Parameter;
				}
			break;
			case CTRL_OP_Set_Target_Inclination: 					//0x03	Set Target Inclination				--SINT16, in Percent with a resolution of 0.1 %
				temp32 = (int16_t)he_uint32(cp_data->pram, 2);
			
				if(if_limit_min_max(temp32, my_flash.ftms.range.incline.min, my_flash.ftms.range.incline.max))
				{
					NRF_LOG_INFO("set inc:%d.%d %%\n", temp32/10, temp32%10);
			
					if(dkn_cmd.set.incline != temp32)
					{
						dkn_cmd.set.incline = temp32;
						dkn_cmd.ctrl.b.incline_changed = 1;
					}
					
					response.result = CTRL_RESULT_Success;
				}
				else
				{
					response.result = CTRL_RESULT_Invalid_Parameter;
				}
			break;
			case CTRL_OP_Set_Target_Resistance_Level: 			//0x04	Set Target Resistance Level		--UINT8, unitless with a resolution of 0.1.
				temp32 = (int16_t)he_uint32(cp_data->pram, 2);

				if(if_limit_min_max(temp32, my_flash.ftms.range.resist.min, my_flash.ftms.range.resist.max))
				{
					NRF_LOG_INFO("resist=%d.%d\n", temp32/10, temp32%10);
			
					temp32 /= 10;
				
					if(temp32 < 1)
						temp32 = 1;
					
					if(dkn_cmd.set.level != temp32)
					{
						dkn_cmd.set.level = temp32;
						dkn_cmd.ctrl.b.level_changed = 1;
					}
				
					response.result = CTRL_RESULT_Success;
				}
				else
				{
					response.result = CTRL_RESULT_Invalid_Parameter;
				}
			break;
			case CTRL_OP_Set_Target_Power: 								//0x05	Set Target Power							--SINT16, in Watt with a resolution of 1 W.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Target_Heart_Rate: 						//0x06	Set Target Heart Rate					--UINT8, in BPM with a resolution of 1 BPM.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Start_or_Resume: 									//0x07	Start or Resume
				NRF_LOG_INFO("req start\n");
			
				if(dkn_cmd.sta.b_sport == 0)
				{
					dkn_cmd.ctrl.b.req_start = 1;
					dkn_cmd.ctrl.b.req_pause = 0;
					dkn_cmd.ctrl.b.req_stop = 0;
					response.result = CTRL_RESULT_Success;
				}
				else
				{
					response.result = CTRL_RESULT_Operation_Failed;
				}
			break;
			case CTRL_OP_Stop_or_Pause: 										//0x08	Stop or Pause									--UINT8, see ble_ftms_machine_status_stop_or_pause_def
				NRF_LOG_INFO("req stop or pause:");
				if(cp_data->pram[0] == STATUS_STOP_OR_PAUSE_STOP)
				{
					NRF_LOG_INFO("stop\n");
					if(dkn_cmd.sta.b_sport)
					{
						dkn_cmd.ctrl.b.req_start = 0;
						dkn_cmd.ctrl.b.req_pause = 0;
						dkn_cmd.ctrl.b.req_stop = 1;
						response.result = CTRL_RESULT_Success;
					}
					else
					{
						response.result = CTRL_RESULT_Operation_Failed;
					}
					
				}
				else if(cp_data->pram[0] == STATUS_STOP_OR_PAUSE_PAUSE)
				{
					NRF_LOG_INFO("pause\n");
					if(dkn_cmd.sta.b_sport)
					{
						dkn_cmd.ctrl.b.req_start = 0;
						dkn_cmd.ctrl.b.req_pause = 1;
						dkn_cmd.ctrl.b.req_stop = 0;
						response.result = CTRL_RESULT_Success;
					}
					else
					{
						response.result = CTRL_RESULT_Operation_Failed;
					}
				}
				else
				{
					NRF_LOG_INFO("res\n");

					response.result = CTRL_RESULT_Invalid_Parameter;
				}
			break;
			case CTRL_OP_Set_Targeted_Expended_Energy: 		//0x09	Set Targeted Expended Energy	--UINT16, in Calories with a resolution of 1Calorie.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Number_of_Steps: 		//0x0A	Set Targeted Number of Steps	--UINT16, in Stepswith a resolution of 1 Step.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Number_of_Strides: 	//0x0B	Set Targeted Number of Strides--UINT16, in Stridewith a resolution of 1 Stride.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Distance: 						//0x0C	Set Targeted Distance					--UINT24, in Meterswith a resolution of 1 Meter.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Training_Time: 			//0x0D	Set Targeted Training Time		--UINT16, in Seconds with a resolution of 1Second.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Time_in_2_Heart_Rate_Zones: 			//0x0E	Set Targeted Time in Two Heart Rate Zones
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Time_in_3_Heart_Rate_Zones: 			//0x0F	Set Targeted Time in Three Heart Rate Zones
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Time_in_5_Heart_Rate_Zones: 			//0x10	Set Targeted Time in Five Heart Rate Zones
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Indoor_Bike_Simulation_Parameters: 			//0x11	Set Indoor Bike Simulation Parameters	
				NRF_LOG_INFO("set bike data:");

				temp32 = (int16_t)he_uint32(&cp_data->pram[2], 2);
			
				NRF_LOG_INFO("resist=%d.%02d\n", temp32/100, temp32%100);
			
				temp32 /= 10;
			
				if(temp32 < 10)
					temp32 = 10;
			
				if(if_limit_min_max(temp32, my_flash.ftms.range.resist.min, my_flash.ftms.range.resist.max))
				{
					temp32 /= 10;
					if(dkn_cmd.set.level != temp32)
					{
						dkn_cmd.set.level = temp32;
						dkn_cmd.ctrl.b.level_changed = 1;
					}

					response.result = CTRL_RESULT_Success;
				}
				else
				{
					response.result = CTRL_RESULT_Invalid_Parameter;
				}
			break;
			case CTRL_OP_Set_Wheel_Circumference: 					//0x12	Set Wheel Circumference				--UINT16, in Millimeters with resolution of 0.1 Millimeter
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Spin_Down_Control: 								//0x13	Spin Down Control
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Set_Targeted_Cadence: 						//0x14	Set Targeted Cadence					--UINT16, in 1/minute with a resolution of 0.51/minute.
				response.result = CTRL_RESULT_Control_Not_Permitted;
			break;
			case CTRL_OP_Request_Control:break;										//0x00	Request Control
			default:
				response.result = CTRL_RESULT_Op_Code_not_supported;
			break;
		}
	}
	else
	{
		response.result = CTRL_RESULT_Control_Not_Permitted;
	}
	
	
	
	response.op_code = cp_data->op_code;

	ble_ftms_control_point_send(p_ftms, &response);
}



/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	
		// Create timers

	
	err_code = app_timer_create(&m_timer_500ms_running_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_500ms_running_handler);
	APP_ERROR_CHECK(err_code);
	
	
	//m_timer_sys_reset_delay_id
	err_code = app_timer_create(&m_timer_sys_reset_delay_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_system_reset_delay_handler);
	APP_ERROR_CHECK(err_code);
	
	
	err_code = app_timer_start(m_timer_500ms_running_id, ANT_500ms_DELL_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle_peripheral == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
		
		ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

		if(my_flash.name_len && (my_flash.device_name[0] != 0))
		{
			
			err_code = sd_ble_gap_device_name_set(&sec_mode,
																						(const uint8_t *)my_flash.device_name,
																							(my_flash.name_len > 20)?(20):(my_flash.name_len));
		}
		else
		{
			err_code = sd_ble_gap_device_name_set(&sec_mode,
																						(const uint8_t *)DEVICE_NAME,
																						strlen(DEVICE_NAME));
		}
//		NRF_LOG_INFO("sd_ble_gap_device_name_set:%d", err_code);
    APP_ERROR_CHECK(err_code);

//    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
//    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

//    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
//    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
//    gap_conn_params.slave_latency     = SLAVE_LATENCY;
//    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
		
		gap_conn_params.min_conn_interval = m_scan.conn_params.min_conn_interval;
    gap_conn_params.max_conn_interval = m_scan.conn_params.max_conn_interval;
    gap_conn_params.slave_latency     = m_scan.conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = m_scan.conn_params.conn_sup_timeout;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}





/**@snippet [UART Initialization] */

/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t             err_code;
    ble_advdata_t        advdata;
    ble_gap_adv_data_t   advdata_enc;
    ble_gap_adv_params_t adv_params;
    static uint8_t       advdata_buff[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    uint16_t             advdata_buff_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
		
		static uint8_t       advrspdata_buff[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
		uint16_t             advrspdata_buff_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
	
    uint8_t              flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
		ble_advdata_service_data_t 		m_adv_service_data;
    ble_uuid_t adv_uuids[] =
    {
				{FTMS_UUID_SERVICE, BLE_UUID_TYPE_BLE}
    };
		uint8_t m_service_data[3]={BLE_FTMS_ADV_FLAG_AVAILABLE, my_flash.ftms.type.value, 0x00};	

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
		
		m_adv_service_data.service_uuid = FTMS_UUID_SERVICE;
		m_adv_service_data.data.size = 3;
		m_adv_service_data.data.p_data = m_service_data;
		
		advdata.service_data_count = 1;
		advdata.p_service_data_array = &m_adv_service_data;
		

    err_code = ble_advdata_encode(&advdata, advdata_buff, &advdata_buff_len);
		NRF_LOG_INFO("ble_advdata_encode:%d", err_code);
    APP_ERROR_CHECK(err_code);

    m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

    memset(&advdata_enc, 0, sizeof(advdata_enc));

    advdata_enc.adv_data.p_data = advdata_buff;
    advdata_enc.adv_data.len    = advdata_buff_len;
		
		
		//========
		advrspdata_buff_len = my_flash.name_len + 2;
		advrspdata_buff[0] = my_flash.name_len + 1;
		advrspdata_buff[1] = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
		memcpy(&advrspdata_buff[2], my_flash.device_name, my_flash.name_len);
		
		advdata_enc.scan_rsp_data.p_data = advrspdata_buff;
		advdata_enc.scan_rsp_data.len = advrspdata_buff_len;

    // Initialise advertising parameters (used when starting advertising).
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;
    adv_params.duration        = APP_ADV_DURATION;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &advdata_enc, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_db_discovery_t const * p_db = (ble_db_discovery_t *)p_evt->params.p_db_instance;

    ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
//    ble_rscs_on_db_disc_evt(&m_rscs_c, p_evt);

    if (p_evt->evt_type == BLE_DB_DISCOVERY_AVAILABLE) {
        NRF_LOG_INFO("DB Discovery instance %p available on conn handle: %d",
                     p_db,
                     p_evt->conn_handle);
        NRF_LOG_INFO("Found %d services on conn_handle: %d",
                     p_db->srv_count,
                     p_evt->conn_handle);
    }
}

/**
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
	
	NRF_LOG_INFO("ble_db_discovery_init:%d", err_code);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
	
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
						NRF_LOG_INFO("NRF_BLE_SCAN_EVT_CONNECTING_ERROR:%d", err_code);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
//    ble_uuid_t          target_uuid = 
//    {
//        .uuid = BLE_UUID_HEART_RATE_SERVICE,
//        .type = BLE_UUID_TYPE_BLE
//    };
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
		NRF_LOG_INFO("nrf_ble_scan_init:%d", err_code);
    APP_ERROR_CHECK(err_code);

//    if (strlen(m_target_periph_name) != 0)
//    {
//        err_code = nrf_ble_scan_filter_set(&m_scan, 
//                                           SCAN_NAME_FILTER, 
//                                           m_target_periph_name);
//        APP_ERROR_CHECK(err_code);
//    }

//    err_code = nrf_ble_scan_filter_set(&m_scan, 
//                                       SCAN_UUID_FILTER, 
//                                       &target_uuid);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrf_ble_scan_filters_enable(&m_scan, 
//                                           /*NRF_BLE_SCAN_NAME_FILTER |*/ NRF_BLE_SCAN_UUID_FILTER, 
//                                           false);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
//	NRF_LOG_INFO("nrf_ble_scan_start:%d", err_code);
    APP_ERROR_CHECK(err_code);

		Sys.status.ble_is_scaning = true;
    NRF_LOG_INFO("Scanning");
}

//初始化mftms结构体并添加FTMS服务
void mftms_svc_add(void)
{
	ret_code_t err_code;
	ble_ftms_init_t		 ftms_init;
	
	if(my_flash.ftms.enable == 0)
		return;
	
	
	memset(&mftms, 0, sizeof(mftms));
	memset(&ftms_init, 0, sizeof(ftms_init));

	//从flash将机型信息拷贝到mftms结构体
	ftms_init.feature.fitness_machine = my_flash.ftms.feature.fitness_machine;
	ftms_init.feature.target_setting = my_flash.ftms.feature.target_setting;
	
	memcpy(&ftms_init.speed_range, &my_flash.ftms.range.speed, sizeof(ftms_init.speed_range));
	memcpy(&ftms_init.incline_range, &my_flash.ftms.range.incline, sizeof(ftms_init.incline_range));
	memcpy(&ftms_init.power_range, &my_flash.ftms.range.power, sizeof(ftms_init.power_range));
	memcpy(&ftms_init.resist_range, &my_flash.ftms.range.resist, sizeof(ftms_init.resist_range));
	memcpy(&ftms_init.heart_rate_range, &my_flash.ftms.range.heartrate, sizeof(ftms_init.heart_rate_range));
	
	
	/*Test*/
//	my_flash.ftms.type.value = 0;
//	//my_flash.ftms.type.b.treadmill_support = 1;
//	my_flash.ftms.type.b.cross_trainer_support = 1;
//	my_flash.ftms.type.b.indoor_bike_support = 1;
	
//	ftms_init.speed_range.min = 100;
//	ftms_init.speed_range.max = 1600;
//	ftms_init.speed_range.inc = 10;
//	
//	ftms_init.incline_range.min = 0;
//	ftms_init.incline_range.max = 150;
//	ftms_init.incline_range.inc = 10;
//	
//	ftms_init.heart_rate_range.min = 40;
//	ftms_init.heart_rate_range.max = 220;
//	ftms_init.heart_rate_range.min = 1;

//	ftms_init.resist_range.min = 10;
//	ftms_init.resist_range.max = 200;
//	ftms_init.resist_range.inc = 10;
//	ftms_init.power_range.min = 30;
//	ftms_init.power_range.max = 1500;
//	ftms_init.power_range.inc = 1;
//	
//	ftms_init.feature.fitness_machine = 0xff;
//	ftms_init.feature.target_setting = (1<<2) | (1<<3);
	/*Test end*/
	
	
	if(ftms_init.speed_range.max)
		ftms_init.range_suport.b.supported_speed_range = 1;
	if(ftms_init.incline_range.max)
		ftms_init.range_suport.b.supported_inclination_range = 1;
	if(ftms_init.power_range.max)
		ftms_init.range_suport.b.supported_power_range = 1;
	if(ftms_init.resist_range.max)
		ftms_init.range_suport.b.supported_resistance_level_range = 1;
	if(ftms_init.heart_rate_range.max)
		ftms_init.range_suport.b.supported_heart_rate_range = 1;
	
	if(my_flash.ftms.type.b.treadmill_support)
	{
		memset(&treadmill, 0, sizeof(treadmill));
		
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_AVGSPEED)									//Average Speed Supported
			treadmill.flags.b.avg_speed_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_CADENCE)									//Cadence Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_TOTAL_DISTANCE)						//Total Distance Supported
			treadmill.flags.b.total_distance_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_INCLINATION)							//Inclination Supported
			treadmill.flags.b.inclination_and_ramp_angle_setting_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELEVATION_GAIN)						//Elevation Gain Supported
			treadmill.flags.b.elevation_gain_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_PACE)											//Pace Supported
			treadmill.flags.b.inst_pace_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STEP_COUNT)								//Step Count Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_RESISTANCE_LEVEL)					//Resistance Level Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STRIDE_COUNT)							//Stride Count Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_EXPENDED_ENERGY)  				//Expended Energy Supported
			treadmill.flags.b.expended_energy_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_HEART_RATE_MEASUREMENT)		//Heart Rate Measurement Supported
			treadmill.flags.b.heart_rate_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_METABOLIC_EQUIVALENT)			//Metabolic Equivalent Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELAPSED_TIME)							//Elapsed Time Supported
			treadmill.flags.b.elapsed_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_REMAINING_TIME)						//Remaining Time Supported
			treadmill.flags.b.remaining_time_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_POWER_MEASUREMENT)				//Power Measurement Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_FORCE_ON_BELT_AND_POWER_OUTPUT)			//Force on Belt and Power Output Supported
			treadmill.flags.b.forceonbelt_powerout_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_USER_DATA_RETENTION)			//User Data Retention Supported
		
		
		ftms_init.initial_treadmill = &treadmill;
	}
	
	if(my_flash.ftms.type.b.cross_trainer_support)
	{
		memset(&cross_trainer, 0, sizeof(cross_trainer));
		
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_AVGSPEED)									//Average Speed Supported
			cross_trainer.flags.b.avg_speed_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_CADENCE)									//Cadence Supported
//			cross_trainer.flags.b.step_count_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_TOTAL_DISTANCE)						//Total Distance Supported
			cross_trainer.flags.b.total_distance_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_INCLINATION)							//Inclination Supported
			cross_trainer.flags.b.inclination_and_ramp_angle_setting_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELEVATION_GAIN)						//Elevation Gain Supported
			cross_trainer.flags.b.elevation_gain_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_PACE)											//Pace Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STEP_COUNT)								//Step Count Supported
			cross_trainer.flags.b.step_count_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_RESISTANCE_LEVEL)					//Resistance Level Supported
			cross_trainer.flags.b.resistance_level_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STRIDE_COUNT)							//Stride Count Supported
			cross_trainer.flags.b.stride_count_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_EXPENDED_ENERGY)  				//Expended Energy Supported
			cross_trainer.flags.b.expended_energy_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_HEART_RATE_MEASUREMENT)		//Heart Rate Measurement Supported
			cross_trainer.flags.b.heart_rate_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_METABOLIC_EQUIVALENT)			//Metabolic Equivalent Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELAPSED_TIME)							//Elapsed Time Supported
			cross_trainer.flags.b.elapsed_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_REMAINING_TIME)						//Remaining Time Supported
			cross_trainer.flags.b.remaining_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_POWER_MEASUREMENT)				//Power Measurement Supported
			cross_trainer.flags.b.instantaneous_power_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_FORCE_ON_BELT_AND_POWER_OUTPUT)			//Force on Belt and Power Output Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_USER_DATA_RETENTION)			//User Data Retention Supported
		
		
		ftms_init.initial_cross_trainer = &cross_trainer;
	}
	
	if(my_flash.ftms.type.b.rower_support)
	{
		memset(&rower, 0, sizeof(rower));
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_AVGSPEED)									//Average Speed Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_CADENCE)									//Cadence Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_TOTAL_DISTANCE)						//Total Distance Supported
			rower.flags.b.total_distance_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_INCLINATION)							//Inclination Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELEVATION_GAIN)						//Elevation Gain Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_PACE)											//Pace Supported
			rower.flags.b.inst_pace_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STEP_COUNT)								//Step Count Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_RESISTANCE_LEVEL)					//Resistance Level Supported
			rower.flags.b.resistance_level_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STRIDE_COUNT)							//Stride Count Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_EXPENDED_ENERGY)  				//Expended Energy Supported
			rower.flags.b.expended_energy_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_HEART_RATE_MEASUREMENT)		//Heart Rate Measurement Supported
			rower.flags.b.heart_rate_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_METABOLIC_EQUIVALENT)			//Metabolic Equivalent Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELAPSED_TIME)							//Elapsed Time Supported
			rower.flags.b.elapsed_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_REMAINING_TIME)						//Remaining Time Supported
			rower.flags.b.remaining_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_POWER_MEASUREMENT)				//Power Measurement Supported
			rower.flags.b.inst_power_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_FORCE_ON_BELT_AND_POWER_OUTPUT)			//Force on Belt and Power Output Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_USER_DATA_RETENTION)			//User Data Retention Supported
		
		ftms_init.initial_rower = &rower;
	}
	
	if(my_flash.ftms.type.b.indoor_bike_support)
	{
		memset(&bike, 0, sizeof(bike));
		
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_AVGSPEED)									//Average Speed Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_CADENCE)									//Cadence Supported
			bike.flags.b.inst_cadence = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_TOTAL_DISTANCE)						//Total Distance Supported
			bike.flags.b.total_distance_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_INCLINATION)							//Inclination Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELEVATION_GAIN)						//Elevation Gain Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_PACE)											//Pace Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STEP_COUNT)								//Step Count Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_RESISTANCE_LEVEL)					//Resistance Level Supported
			bike.flags.b.resist_level_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_STRIDE_COUNT)							//Stride Count Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_EXPENDED_ENERGY)  				//Expended Energy Supported
			bike.flags.b.expended_energy_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_HEART_RATE_MEASUREMENT)		//Heart Rate Measurement Supported
			bike.flags.b.heart_rate_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_METABOLIC_EQUIVALENT)			//Metabolic Equivalent Supported
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_ELAPSED_TIME)							//Elapsed Time Supported
			bike.flags.b.elapsed_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_REMAINING_TIME)						//Remaining Time Supported
			bike.flags.b.remaining_time_present = 1;
		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_POWER_MEASUREMENT)				//Power Measurement Supported
			bike.flags.b.inst_power_present = 1;
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_FORCE_ON_BELT_AND_POWER_OUTPUT)			//Force on Belt and Power Output Supported
//		if(ftms_init.feature.fitness_machine & BLE_FTMS_FEATURE_USER_DATA_RETENTION)			//User Data Retention Supported
		
		ftms_init.initial_indoorbike = &bike;
	}
	
	ftms_training_status.flags.b.training_status_string_present = 1;
	ftms_training_status.status = FTMS_TS_Idle;
	
	ftms_machine_status.op_data = STATUS_OP_Stop_or_pause;
	ftms_machine_status.parameter[0] = STATUS_STOP_OR_PAUSE_STOP;
	ftms_machine_status.par_len = 1;
	
	ftms_init.initial_training_status = &ftms_training_status;
	ftms_init.initial_machine_status = &ftms_machine_status;
	
	//control point回调
	ftms_init.control_point_evt_handle = ftms_contol_point_data_handler;
	
	//添加FTMS服务
	err_code = ble_ftms_init(&mftms, &ftms_init);
	NRF_LOG_INFO("ble_ftms_init:%X", err_code);
	APP_ERROR_CHECK(err_code);
	
	
	treadmill.ramp_angle = 0x7FFF;
}


/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Heart Rate and Device Information services.
 */
static void services_init(void)
{
    uint32_t           err_code;
//    ble_hrs_init_t     hrs_init;
		ble_nus_init_t		 nus_init;
    ble_dis_init_t     dis_init;
		ble_hrs_init_t		 hrs_init;
    nrf_ble_qwr_init_t qwr_init = {0};
		
		uint8_t            body_sensor_location;
		
//    uint8_t            body_sensor_location;

    // Initialize the Queued Write module.
    qwr_init.error_handler = nrf_qwr_error_handler;

		for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }


    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

		if(my_flash.ftms.string.dis_manufacturer_name.Length && my_flash.ftms.string.dis_manufacturer_name.Value[0])
		{
			dis_init.manufact_name_str.p_str = my_flash.ftms.string.dis_manufacturer_name.Value;
			dis_init.manufact_name_str.length = my_flash.ftms.string.dis_manufacturer_name.Length;
		}
    else 
			ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
		
		if(my_flash.ftms.string.dis_model_nb.Length && my_flash.ftms.string.dis_model_nb.Value[0])
		{
			dis_init.model_num_str.p_str = my_flash.ftms.string.dis_model_nb.Value;
			dis_init.model_num_str.length = my_flash.ftms.string.dis_model_nb.Length;
		}
    else 
			ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);
		
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FW_STR);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
		 // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_OTHER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = false;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);
		
//====================================================	FTMS	
		mftms_svc_add();
		
//====================================================			
		memset(&nus_init, 0, sizeof(nus_init));
		
		nus_init.data_handler = nus_data_handler;
		err_code = ble_nus_init(&m_nus, &nus_init);
		NRF_LOG_INFO("ble_nus_init:%X", err_code);
		APP_ERROR_CHECK(err_code);
}


///**@brief Connection Parameters Module handler.
// *
// * @details This function will be called for all events in the Connection Parameters Module which
// *          are passed to the application.
// *          @note All this function does is to disconnect. This could have been done by simply
// *                setting the disconnect_on_fail config parameter, but instead we use the event
// *                handler mechanism to demonstrate its use.
// *
// * @param[in]   p_evt   Event received from the Connection Parameters Module.
// */
//static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
//{
//    uint32_t err_code;

//    switch (p_evt->evt_type)
//    {
//        case BLE_CONN_PARAMS_EVT_FAILED:
//            err_code = sd_ble_gap_disconnect(m_conn_handle_peripheral, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//						NRF_LOG_INFO("on_conn_params");
//            APP_ERROR_CHECK(err_code);
//            break;

//        default:
//            // No implementation needed.
//            break;
//    }
//}
uint32_t DKN_nus_Send(uint8_t* buf, uint16_t len)
{
	ret_code_t err_code;
	err_code = ble_nus_data_send(&m_nus, buf, &len, m_conn_handle_peripheral);
			
//	NRF_LOG_INFO("NUS SEND:%d", err_code);
	return err_code;
}


void DKN_Send_Ftms_machine_status(void)
{
	ble_ftms_machine_status_send(&mftms, &ftms_machine_status);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
	uint8_t tx_buf[10],i;
	if (p_evt->type == BLE_NUS_EVT_RX_DATA)
	{
		NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
		
		uart_auto_hold();
		
		uart_send_array(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
		
//		Sys.time.uart_delay = 0;
		
		uart_send_cmd_after_nus_write();
	}

}

/**@brief Function for handling the Heart Rate Service Client
 *        Running Speed and Cadence Service Client.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID;
//		cp_init.start_on_notify_cccd_handle = m_nus.rx_handles.cccd_handle;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
	NRF_LOG_INFO("ble_conn_params_init:%d", err_code);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
//	uint32_t               err_code;
	ant_hrm_profile_t * p_profile = ( ant_hrm_profile_t *)p_context;
	uint32_t tmep32;
	uint16_t id;
	int8_t rssi;
	uint8_t i,pulse;
	bool is_new_dev;

    if (p_ant_evt->channel == p_profile->channel_number)
    {
        switch (p_ant_evt->event)
        {
            case EVENT_RX:
                if (p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID
                 || p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_ACKNOWLEDGED_DATA_ID
                 || p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BURST_DATA_ID)
                {

									memcpy((uint8_t*)&tmep32, &p_ant_evt->message.ANT_MESSAGE_aucExtData[0], 4);
									id = (uint16_t)tmep32;
									//memcpy((uint8_t*)&rssi, &p_ant_evt->message.ANT_MESSAGE_aucExtData[4], 3);
									rssi = (int8_t)p_ant_evt->message.ANT_MESSAGE_aucExtData[5];
									pulse = p_ant_evt->message.ANT_MESSAGE_aucPayload[7];
									
									if(Ant_Hrm.status == 0)
									{
										is_new_dev = true;
										if(Ant_Hrm.num)
										{
											for(i=0;i<Ant_Hrm.num;i++)
											{
												if(Ant_Hrm.device[i].id == id)
												{
													is_new_dev = false;
													Ant_Hrm.device[i].rssi = rssi;
													Ant_Hrm.device[i].pulse = pulse;
												}
											}
										}
										
										if(is_new_dev)
										{
											if(Ant_Hrm.num < 10)
											{
												Ant_Hrm.device[Ant_Hrm.num].id = id;
												Ant_Hrm.device[Ant_Hrm.num].rssi = rssi;
												Ant_Hrm.device[Ant_Hrm.num].pulse = pulse;
												
												NRF_LOG_INFO("Ant New[%d], id: %d, rssi: %d", Ant_Hrm.num, Ant_Hrm.device[Ant_Hrm.num].id, Ant_Hrm.device[Ant_Hrm.num].rssi);
												
												Ant_Hrm.num++;
											}
										}
									}
									else
									{
										if(Ant_Hrm.select_id == id)
										{
											Ant_Hrm.pulse = pulse;
											Ant_Hrm.lost_time = 0;
											uart_has_hr_recv(1);
											
											NRF_LOG_INFO("Ant Hr:%d", Ant_Hrm.pulse);
										}
									}
									
//									my_ant_hrm_get_data(id, rssi, p_ant_evt->message.ANT_MESSAGE_aucPayload[7]);
//									if((Dev_Id_Select==0) || (Dev_Id_Select == (uint16_t)id))
//									{
//										NRF_LOG_INFO("id:%d, rssi:%d, pulse: %d", (uint16_t)id, rssi, p_ant_evt->message.ANT_MESSAGE_aucPayload[7]);
////									
////										
//										if((Dev_Id_Select==0) && (rssi > -50))
//										{
//											if(Dev_Id_Select == 0)
//											{
////												err_code = sd_ant_channel_close(0);
////												NRF_LOG_INFO("sd_ant_channel_close:%04X", err_code);
////												
////												ant_hrm_rx_start();
//											}
//											
//											Dev_Id_Select = (uint16_t)id;
//										}
//									}

//                    disp_message_decode(p_profile, p_ant_evt->message.ANT_MESSAGE_aucPayload);
                }
                break;
						
            default:
                break;
        }
    }
			
//		NRF_LOG_INFO("ant_evt:%d\r", p_ant_evt->event);
		
    if (p_ant_evt->event != EVENT_CHANNEL_CLOSED)
    {
        return;
    }
    if (m_conn_handle_peripheral != BLE_CONN_HANDLE_INVALID)
    {
//        ant_hrm_rx_start();
    }
    else
    {
//        ant_and_adv_start();
    }
		
		
}


/**@brief Handle received ANT+ HRM data.
 *
 * @param[in]   p_profile       Pointer to the ANT+ HRM profile instance.
 * @param[in]   event           Event related with ANT+ HRM Display profile.
 */
//static void ant_hrm_evt_handler(ant_hrm_profile_t * p_profile, ant_hrm_evt_t event)
//{
//    static uint32_t     s_previous_beat_count  = 0;    // Heart beat count from previously received page
//    uint32_t            err_code;
////    uint16_t            beat_time              = p_profile->page_0.beat_time;
//    uint32_t            beat_count             = p_profile->page_0.beat_count;
////    uint32_t            computed_heart_rate    = p_profile->page_0.computed_heart_rate;

//    switch (event)
//    {
//        case ANT_HRM_PAGE_0_UPDATED:
//            /* fall through */
//        case ANT_HRM_PAGE_1_UPDATED:
//            /* fall through */
//        case ANT_HRM_PAGE_2_UPDATED:
//            /* fall through */
//        case ANT_HRM_PAGE_3_UPDATED:
//            break;
//        case ANT_HRM_PAGE_4_UPDATED:

//            // Ensure that there is only one beat between time intervals.
//            if ((beat_count - s_previous_beat_count) == 1)
//            {
////                uint16_t prev_beat = p_profile->page_4.prev_beat;

//                // Subtracting the event time gives the R-R interval
////                ble_hrs_rr_interval_add(&m_hrs, beat_time - prev_beat);
//            }

//            s_previous_beat_count = beat_count;
//            break;

//        default:
//            break;
//    }

//    // Notify the received heart rate measurement
////    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, computed_heart_rate);
//    if ((err_code != NRF_SUCCESS) &&
//        (err_code != NRF_ERROR_INVALID_STATE) &&
//        (err_code != NRF_ERROR_RESOURCES) &&
//        (err_code != NRF_ERROR_BUSY) &&
//        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//       )
//    {
//			NRF_LOG_INFO("if here");
////        APP_ERROR_HANDLER(err_code);
//    }
//}
/**@brief Function for checking whether a link already exists with a newly connected peer.
 *
 * @details This function checks whether the newly connected device is already connected.
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
//static bool is_already_connected(ble_gap_addr_t const * p_connected_adr)
//{
//    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
//    {
//        if (m_connected_peers[i].is_connected)
//        {
//            if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type)
//            {
//                if (memcmp(m_connected_peers[i].address.addr,
//                           p_connected_adr->addr,
//                           sizeof(m_connected_peers[i].address.addr)) == 0)
//                {
//                    return true;
//                }
//            }
//        }
//    }
//    return false;
//}


/** @brief Function for handling a numeric comparison match request. */
//static void on_match_request(uint16_t conn_handle, uint8_t role)
//{
//    // Mark the appropriate conn_handle as pending. The rest is handled on button press.
//    NRF_LOG_INFO("Press Button 1 to confirm, Button 2 to reject");
//    if (role == BLE_GAP_ROLE_CENTRAL)
//    {
//        m_conn_handle_num_comp_central = conn_handle;
//    }
//    else if (role == BLE_GAP_ROLE_PERIPH)
//    {
//        m_conn_handle_peripheral = conn_handle;
//    }
//}
/**@brief Handles events coming from the Heart Rate central module.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_hrs_c = p_hrs_c_evt->conn_handle;
                NRF_LOG_INFO("HRS discovered on conn_handle 0x%x", m_conn_handle_hrs_c);

                //filter_settings_change();

                err_code = ble_hrs_c_handles_assign(p_hrs_c,
                                                    m_conn_handle_hrs_c,
                                                    &p_hrs_c_evt->params.peer_db);
							
							NRF_LOG_INFO("ble_hrs_c_handles_assign:%d", err_code);
                APP_ERROR_CHECK(err_code);
                // Initiate bonding.
//                err_code = pm_conn_secure(m_conn_handle_hrs_c, false);
//							NRF_LOG_INFO("pm_conn_secure:%d", err_code);
//                if (err_code != NRF_ERROR_BUSY)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }

                // Heart rate service discovered. Enable notification of Heart Rate Measurement.
                err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
								NRF_LOG_INFO("ble_hrs_c_hrm_notif_enable:%d", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_HRS_C_EVT_DISCOVERY_COMPLETE

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
//            ret_code_t err_code;

						NRF_LOG_INFO("BLE Hr:%d", p_hrs_c_evt->params.hrm.hr_value);

						Ble_Hr.pulse = p_hrs_c_evt->params.hrm.hr_value;
					
						uart_has_hr_recv(1);
//            err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, p_hrs_c_evt->params.hrm.hr_value);
//            if ((err_code != NRF_SUCCESS) &&
//                (err_code != NRF_ERROR_INVALID_STATE) &&
//                (err_code != NRF_ERROR_RESOURCES) &&
//                (err_code != NRF_ERROR_BUSY) &&
//                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//                )
//            {
//                APP_ERROR_HANDLER(err_code);
//            }
        } break; // BLE_HRS_C_EVT_HRM_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for assigning new connection handle to the available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
					
						NRF_LOG_INFO("nrf_ble_qwr_conn_handle_assign:%d",err_code);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, const data_t * p_advdata, uint8_t * p_typedata, uint16_t * p_datalen)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
//            p_typedata   = &p_data[index + 2];
            *p_datalen = field_length - 1;
						memcpy(p_typedata, &p_data[index + 2], *p_datalen);
					
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for searching a UUID in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * UUID in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   uuid_to_find   UUIID to search.
 * @return   true if the given UUID was found, false otherwise.
 */
static bool find_adv_uuid(const ble_gap_evt_adv_report_t *p_adv_report, const uint16_t uuid_to_find)
{
    uint32_t err_code;
    data_t   adv_data;
//    data_t   type_data;
		uint8_t data[128];
		uint16_t datalen;

    // Initialize advertisement report for parsing.
    adv_data.p_data     = (uint8_t *)p_adv_report->data.p_data;
    adv_data.data_len   = p_adv_report->data.len;
	
//	NRF_LOG_INFO("adv data befor:\r\n");
//							NRF_LOG_RAW_HEXDUMP_INFO(adv_data.p_data , adv_data.data_len);
	
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                &adv_data,
                                data,
																&datalen);

    if (err_code != NRF_SUCCESS)
    {
        // Look for the services in 'complete' if it was not found in 'more available'.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                    &adv_data,
                                    data,
																		&datalen);

        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit.
            return false;
        }
    }
//NRF_LOG_INFO("adv data:\r\n");
//							NRF_LOG_RAW_HEXDUMP_INFO(adv_data.p_data , adv_data.data_len);
    // Verify if any UUID match the given UUID.
    for (uint32_t u_index = 0; u_index < (datalen / UUID16_SIZE); u_index++)
    {
        uint16_t    extracted_uuid;

        UUID16_EXTRACT(&extracted_uuid, &data[u_index * UUID16_SIZE]);

        if (extracted_uuid == uuid_to_find)
        {
            return true;
        }
    }
    return false;
}

/**@brief Function for handling BLE Stack events that are related to central application.
 *
 * @details This function keeps the connection handles of central application up-to-date. It
 * parses scanning reports, initiates a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report the central application's activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              must be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t            err_code;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected (HR or RSC), initiate DB
        //  discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Central connected");
						Sys.status.ble_is_scaning = false;
            // If no Heart Rate sensor or RSC sensor is currently connected, try to find them on this peripheral.
            if (m_conn_handle_hrs_c  == BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("Attempt to find HRS or RSC on conn_handle 0x%x", p_gap_evt->conn_handle);
									
								Sys.status.ble_master_is_connected = true;
								Sys.status.ble_is_master_conn_ing = false;
							
								uart_conn_sta_changed();
							
                err_code = ble_db_discovery_start(&m_db_discovery[0], p_gap_evt->conn_handle);
                if (err_code == NRF_ERROR_BUSY)
                {
                    err_code = ble_db_discovery_start(&m_db_discovery[1], p_gap_evt->conn_handle);
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    APP_ERROR_CHECK(err_code);
                }
            }

            // Assign connection handle to the QWR module.
            multi_qwr_conn_handle_assign(p_gap_evt->conn_handle);

            // Update LEDs status, and check whether to look for more peripherals to connect to.
//            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
//                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
//                bsp_board_led_on(CENTRAL_SCANNING_LED);
//                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the status of LEDs, and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
					NRF_LOG_INFO("CEN Disconnect");
            if (p_gap_evt->conn_handle == m_conn_handle_hrs_c)
            {
                NRF_LOG_INFO("HRS central disconnected (reason: %d)",
                             p_gap_evt->params.disconnected.reason);

                m_conn_handle_hrs_c = BLE_CONN_HANDLE_INVALID;
                Ble_Hr.rssi = -127;
								Ble_Hr.pulse = 0;
								Ble_Hr.send_empty_times = 4;
								Ble_Hr.comfirm_time = 0;
							
								Sys.status.ble_master_is_connected = false;
							
								uart_conn_sta_changed();
								uart_has_hr_recv(3);
//                err_code = nrf_ble_scan_filter_set(&m_scan, 
//                                                   SCAN_UUID_FILTER, 
//                                                   &m_adv_uuids[HART_RATE_SERVICE_UUID_IDX]);
//                APP_ERROR_CHECK(err_code);
            }
//            if (p_gap_evt->conn_handle == m_conn_handle_rscs_c)
//            {
//                NRF_LOG_INFO("RSC central disconnected (reason: %d)",
//                             p_gap_evt->params.disconnected.reason);

//                m_conn_handle_rscs_c = BLE_CONN_HANDLE_INVALID;

//                err_code = nrf_ble_scan_filter_set(&m_scan, 
//                                                   SCAN_UUID_FILTER, 
//                                                   &m_adv_uuids[RSCS_SERVICE_UUID_IDX]);
//                APP_ERROR_CHECK(err_code);
//            }

            if (m_conn_handle_hrs_c  == BLE_CONN_HANDLE_INVALID)
            {
                // Start scanning.
							if(Sys.ctrl.ble_scan_enable)
                scan_start();

                // Update LEDs status.
//                bsp_board_led_on(CENTRAL_SCANNING_LED);
            }

//            if (ble_conn_state_central_conn_count() == 0)
//            {
//                bsp_board_led_off(CENTRAL_CONNECTED_LED);
//            }
        } break; // BLE_GAP_EVT_DISCONNECTED
				case BLE_GAP_EVT_ADV_REPORT:
        {
					if(p_gap_evt->params.adv_report.rssi > - 70)
					{
						if(find_adv_uuid(&p_gap_evt->params.adv_report, BLE_UUID_HEART_RATE_SERVICE))//广播信息中包含有心率服务
						{
							if(Ble_Hr.rssi < p_gap_evt->params.adv_report.rssi)
							{
								Ble_Hr.rssi = p_gap_evt->params.adv_report.rssi;
								memcpy(&Ble_Hr.addr, &p_gap_evt->params.adv_report.peer_addr, sizeof(ble_gap_addr_t));
							}
							
							
							
							NRF_LOG_INFO("Find hrs dev:%d", p_gap_evt->params.adv_report.rssi);
							NRF_LOG_HEXDUMP_INFO(p_gap_evt->params.adv_report.peer_addr.addr, 6);
							
							
	//						err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
	//                                          &m_scan.scan_params,
	//                                          &m_scan.conn_params,
	//                                          APP_BLE_CONN_CFG_TAG);


	//						NRF_LOG_ERROR("sd_ble_gap_connect(): 0x%x.\r\n", err_code);
						}
					}

        } break; // BLE_GAP_ADV_REPORT
        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only connection attemps can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("CENTRAL: Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
					NRF_LOG_INFO("sd_ble_gap_conn_param_update:%d", err_code);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_INFO("CENTRAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_INFO("CENTRAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events that involves peripheral applications. Manages the
 * LEDs used to report the status of the peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code = NRF_SUCCESS;

	
//	NRF_LOG_INFO("ble_evt_handler:%04X", p_ble_evt->header.evt_id);
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
				
            APP_ERROR_CHECK(err_code);
            m_conn_handle_peripheral = p_ble_evt->evt.gap_evt.conn_handle;
           
						Sys.status.ble_slave_is_connected = true;
						Sys.status.ble_is_adving = false;
					
						uart_conn_sta_changed();
				
						NRF_LOG_INFO("CONNECTED");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle_peripheral = BLE_CONN_HANDLE_INVALID;

            // Need to close the ANT channel to make it safe to write bonding information to flash
//            err_code = sd_ant_channel_close(ANT_HRMRX_ANT_CHANNEL);
//            APP_ERROR_CHECK(err_code);
						Sys.status.ble_slave_is_connected = false;
				
						uart_conn_sta_changed();
				
						if(Sys.ctrl.ble_adv_enable)
							advertising_start();
						NRF_LOG_INFO("DISCONNECTED");
            // Note: Bonding information will be stored, advertising will be restarted and the
            //       ANT channel will be reopened when ANT event CHANNEL_CLOSED is received.
            break;
				
#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };

            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
						NRF_LOG_INFO("sd_ble_gap_phy_update");
            APP_ERROR_CHECK(err_code);
        } break;
#endif

#ifndef BONDING_ENABLE
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle_peripheral, NULL, NULL, NULL);
				NRF_LOG_INFO("sd_ble_gap_sec_info_reply");
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle_peripheral,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
				NRF_LOG_INFO("sd_ble_gap_sec_params_reply");
            APP_ERROR_CHECK(err_code);
            break;
#endif // BONDING_ENABLE

        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason ==
                BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
            {
                err_code = bsp_indication_set(BSP_INDICATE_IDLE);
							NRF_LOG_INFO("bsp_indication_set");
                APP_ERROR_CHECK(err_code);
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
//                err_code = sd_power_system_off();
//							NRF_LOG_INFO("sd_power_system_off");
//                APP_ERROR_CHECK(err_code);
            }
            break;

#ifndef BONDING_ENABLE
            case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle_peripheral,
                                                     NULL,
                                                     0,
                                                     BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
						NRF_LOG_INFO("sd_ble_gatts_sys_attr_set");
                APP_ERROR_CHECK(err_code);
                break;
#endif // BONDING_ENABLE

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for checking whether a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);
		
		if(p_ble_evt->header.evt_id == 0x10)
		{
			role = p_ble_evt->evt.gap_evt.params.connected.role;
			//NRF_LOG_INFO("role:%d", p_ble_evt->evt.gap_evt.params.connected.role);
			//p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_CENTRAL
			//NRF_LOG_INFO("ble_evt(%d):%08X,role:%d", conn_handle, p_ble_evt->header.evt_id, role);
		}
	
    // Based on the role this device plays in the connection, dispatch to the right handler.
    if (role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt))
    {
        ble_hrs_on_ble_evt(p_ble_evt, &m_hrs);
//        ble_rscs_on_ble_evt(p_ble_evt, &m_rscs);
				ble_nus_on_ble_evt(p_ble_evt, &m_nus);
				ble_ftms_on_ble_evt(p_ble_evt, &mftms);
				
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        ble_hrs_c_on_ble_evt(p_ble_evt, &m_hrs_c);
//        ble_rscs_c_on_ble_evt(p_ble_evt, &m_rscs_c);
        on_ble_central_evt(p_ble_evt);
    }
		
}


#ifdef BONDING_ENABLE
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break;

        default:
            break;
    }
}



#endif // BONDING_ENABLE

//==============================================================================================fds

static void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != NRF_SUCCESS)
            {
                // Initialization failed.
							NRF_LOG_INFO("fds init ok\r\n");
//							b_is_fds_init_ok = true;
            }
            break;
		case FDS_EVT_WRITE:
			if (p_fds_evt->result == NRF_SUCCESS)
			{
			
				NRF_LOG_INFO("fds write ok\r\n");
//				b_write_flash_ok = true;
				
				if(Sys.ctrl.req_reset_after_flash_ok)
				{
					Sys.ctrl.req_reset_after_flash_ok = false;
					system_reset_after_ms(1000);
				}
			}
			break;
		case FDS_EVT_UPDATE:
			if (p_fds_evt->result == NRF_SUCCESS)
			{

				NRF_LOG_INFO("fds updata ok\r\n");
			}
		break;
        default:
            break;
    }
}

void mflash_delete(void)
{
	fds_find_token_t    ftok;
	uint8_t del = 0;
	ret_code_t ret;

	ftok.page=0;
	ftok.p_addr=NULL;
	
	del = 0;
	// Loop and find records with same ID and rec key and mark them as deleted. 
	while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == NRF_SUCCESS)
	{
		fds_record_delete(&record_desc);
		NRF_LOG_INFO("Deleted record ID: %d \r\n",record_desc.record_id);
		del++;
	}
	if(del)
	{
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret = fds_gc();
		if (ret != NRF_SUCCESS)
			NRF_LOG_INFO("fds_gc: %d \r\n",ret);
	}
}

/*
返回：0-成功写入；-1：写入失败；1-不需要写入
*/
int8_t mflash_write(void)
{
//	uint32_t len;
	ret_code_t ret;
	
	if(memcmp(&my_flash_back, &my_flash, sizeof(my_flash_back)) != 0)
	{
	//	len = sizeof(my_flash);
		memcpy(&my_flash_back, &my_flash, sizeof(my_flash_back));

		mflash_delete();

		ret = fds_record_write(&record_desc, &m_dummy_record);
		if (ret != NRF_SUCCESS)
		{
			NRF_LOG_INFO("fds_record_update = %d \r\n",ret);
			return -1;
		}
		NRF_LOG_INFO("Writing Record ID = %d \r\n",record_desc.record_id);
	//	while(write_flag==0);
	//	ret = fds_gc();
	//	NRF_LOG_INFO("fds_gc = %d \r\n",ret);
		return 0;	
	}
	
	return 1;	
}

/*==================================================================
* Function	: void F_Int_BCD(unsigned int data)
* Description	: int型数据BCD码转换
* Input Para	: void
* Output Para	: void
* Return Value  : void
==================================================================*/
static void F_Int_BCD(uint32_t data)
{
	unsigned char i=0;
	data%=100000;
	while(data >= 10000)
		{
		data -= 10000;
		i++;
		}
	BCD._10000 = i;
	i = 0;
	while(data >= 1000)
		{
		data -= 1000;
		i++;
		}
	BCD._1000 = i;
	i = 0;
	while(data >= 100)
		{
		data -= 100;
		i++;
		}
	BCD._100 = i;
	i = 0;
	while(data >= 10)
		{
		data -= 10;
		i++;
		}
	BCD._10 = i;
	BCD._1 = data;
}

int8_t mflash_rewrite(void)
{
	
	memset(&my_flash, 0, sizeof(my_flash));
	NRF_LOG_INFO("fds rewrite\r\n");
	my_flash.name_len = strlen(DEVICE_NAME);
	memcpy(&my_flash.device_name, DEVICE_NAME, my_flash.name_len);
	F_Int_BCD(NRF_FICR->DEVICEADDR[0]);//MAC Addr 低32位
	my_flash.device_name[my_flash.name_len-1] = BCD._1+'0';
	my_flash.device_name[my_flash.name_len-2] = BCD._10+'0';
	my_flash.device_name[my_flash.name_len-3] = BCD._100+'0';
	my_flash.device_name[my_flash.name_len-4] = BCD._1000+'0';

	my_flash.ftms.type.value = 0;
	my_flash.ftms.type.b.treadmill_support = 1;
	
	my_flash.ftms.enable = 1;
	
	my_flash.check = FLASH_CHECK;
	return mflash_write();
}


static void mflash_init(void)
{
	ret_code_t err_code;
	fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
	fds_flash_record_t  flash_record;

	err_code= fds_register(my_fds_evt_handler);
	if (err_code != NRF_SUCCESS)
	{
		NRF_LOG_INFO("fds_register err:%d\r\n",err_code);
		uart_send_array((uint8_t *)err_code, 1);
	}
	err_code = fds_init(); 
	if (err_code != NRF_SUCCESS)
	{
		NRF_LOG_INFO("fds_init err:%d\r\n",err_code);
		uart_send_array((uint8_t *)err_code, 1);
	}
	
 /* Wait for fds to initialize. */
//	while(!b_is_fds_init_ok)
//	{
////		nrf_pwr_mgmt_run();
//	}
	
	NRF_LOG_INFO("fds scan...\r\n");
	// Loop until all records with the given key and file ID have been found.
	while(fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == NRF_SUCCESS)
	{
		err_code = fds_record_open(&record_desc, &flash_record);
		if ( err_code != NRF_SUCCESS)
		{
			NRF_LOG_INFO("fds_record_open err:%d\r\n",err_code);
			
		}
		
		NRF_LOG_INFO("Found Record ID = %d\r\n",record_desc.record_id);
		
		memcpy(&my_flash, (uint8_t *) flash_record.p_data, sizeof(my_flash));
		
		NRF_LOG_INFO("name:%s\r\n",(uint32_t)my_flash.device_name);

		err_code = fds_record_close(&record_desc);
		if (err_code != NRF_SUCCESS)
		{
			NRF_LOG_INFO("fds_record_close err:%d\r\n",err_code);
		}
		
		if(my_flash.check == FLASH_CHECK)
		{
			memcpy(&my_flash_back, &my_flash, sizeof(my_flash_back));
			return ;
		}
	} 
//	else 
//		NRF_LOG_INFO("no record\r\n");

	NRF_LOG_INFO("fds no record:%d\r\n",err_code);
//	uart_send_array((uint8_t *)"rewrite\r\n", 9);
	mflash_rewrite();
}



/**@brief Heart Rate Collector initialization.
 */
static void hrs_c_init(void)
{
    ret_code_t       err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler   = hrs_c_evt_handler;
    hrs_c_init_obj.error_handler = service_error_handler;
    hrs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_hrs_c_init(&m_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief BLE + ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the stack event interrupt.
 */
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
//	NRF_LOG_INFO("nrf_sdh_ble_default_cfg_set:%d", err_code);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);
}


void system_reset_after_ms(uint32_t ms)
{
	ret_code_t err_code;
	
	err_code = app_timer_stop(m_timer_sys_reset_delay_id);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_start(m_timer_sys_reset_delay_id, APP_TIMER_TICKS(ms), NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    // Initialize peripherals
    timers_init();
		uart_init();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    softdevice_setup();

//    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
//    APP_ERROR_CHECK(err_code);

		mflash_init();
	
    // Initialize Bluetooth stack parameters.
		scan_init();
		gap_params_init();
    gatt_init();
		conn_params_init();
    db_discovery_init();
    
		hrs_c_init();
		
    services_init();
		advertising_init();
    

#ifdef BONDING_ENABLE
//    bool erase_bonds = bsp_button_is_pressed(BOND_DELETE_ALL_BUTTON_ID);

//    peer_manager_init(erase_bonds);
//    if (erase_bonds == true)
//    {
//        NRF_LOG_INFO("Bonds erased!");
//    }
#endif // BONDING_ENABLE
    ant_and_adv_start();
		scan_start();
		
//		mLED_init();
		
		Sys.ctrl.ble_adv_enable = true;
		Sys.ctrl.ble_scan_enable = true;
		Sys.ctrl.ant_scan_enable = true;
 // Initialize ANT+ HRM receive channel.
//    err_code = ant_hrm_disp_init(&m_ant_hrm,
//                                 HRM_DISP_CHANNEL_CONFIG(m_ant_hrm),
//                                 ant_hrm_evt_handler);
//		NRF_LOG_INFO("ant_hrm_disp_init:%04X", err_code);
//    APP_ERROR_CHECK(err_code);
		Ble_Hr.rssi = -127;
		Ble_Hr.comfirm_time = 0;
		NRF_LOG_INFO("Domyos Dongle start");
		
		dkn_cmd_init();
    // Enter main loop.
		
    for (;;)
    {
				dkn_cmd_dell_in_main_loop();
			
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}

/**
 * @}
 */

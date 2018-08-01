/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_hids_mouse_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_mouse
 * @brief HID Mouse Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Service for implementing a simple mouse functionality. This application uses the
 * @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device. This implementation of the
 * application will not know whether a connected central is a known device or not.
 */


#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "ble_advertising.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Common addresses definition for temperature sensor. */
#define MGC3130_ADDR        0x42U
#define MGC_3130_TS_PIN     14
#define MGC_3130_RS_PIN     13
#define MGC_3130_LED0_PIN   15 
#define MGC_3130_LED1_PIN   16

#define LOW                 0
#define HIGH                1

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define DEVICE_NAME                     "Mayur_Mouse_Test"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Mayur_Panchal"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   20                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              3                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVENT_DATA_SIZE             /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

#define APP_ADV_FAST_DURATION           3000                                        /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           18000                                       /**< The advertising duration of slow advertising in units of 10 milliseconds. */


BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
BLE_HIDS_DEF(m_hids,                                                                /**< HID service instance. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REP_BUTTONS_LEN,
             INPUT_REP_MOVEMENT_LEN,
             INPUT_REP_MEDIA_PLAYER_LEN);
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static uint8_t m_sample[255];
static uint8_t AirWheelInfo;

//	X-Y-Z Position
union  xyzPosition{
    struct {
        uint16_t x_pos;
        uint16_t y_pos;
        uint16_t z_pos;
    } xyzWord;
    uint8_t  xyzArray[6];
    struct {
        uint8_t  Byte_0;
        uint8_t  Byte_1;
        uint8_t  Byte_2;
        uint8_t  Byte_3;
        uint8_t  Byte_4;
        uint8_t  Byte_5;
    } xyzByte;
} xyzPosition;

//-----------------------------------------------------------------------
//	MGC3130 CMD ID
#define ID_DATA_OUTPUT		0x91
#define ID_FW_VERSION		0x83
//-----------------------------------------------------------------------

#define MASK_GESTURE_RAW	(uint32_t)(0x0001F0FF)	//	Filter mask to remove invalid data into gesture packet
#define MASK_TOUCH_RAW		(uint32_t)(0x00007FFF)	//	Filter mask to remove invalid data into touch packet

#define MASK_FILTER_GESTURE	(uint64_t)(0x0000000000000000)

//----------------------------------------
//	Gesture and Touch Variable
uint32_t LastGesture;
union GestureInfo {
    uint32_t Gesture;
    uint8_t  GestArray[4];
    struct {
        uint8_t  Byte_0;
        uint8_t  Byte_1;
        uint8_t  Byte_2;
        uint8_t  Byte_3;
    } GestureByte;
    struct {
        uint8_t GestureCode			:8;		//	00 -> No Gesture
                                            //	01 -> Garbage Model
                                            //  02 -> Flick West To East
                                            //	03 -> Flick East to West
                                            //	04 -> Flick South to North
                                            //	05 -> Flick North to South
                                            //	06 -> Circle Clockwise
                                            //	07 -> Circle Counter-Clockwise
                                            //  08 -> Wave X
                                            //  09 -> Wave Y
                                            //	64 -> Hold
                                            //  65 -> Edge Flick West To East
                                            //  66 -> Edge Flick East to West
                                            //  67 -> Edge Flick South to North
                                            //  68 -> Edge Flick North to South
                                            //  69 -> Double Flick West To East
                                            //  70 -> Double Flick East to West
                                            //  71 -> Double Flick South to North
                                            //  72 -> Double Flick North to South
                                            //  73 -> Presence
        uint8_t Reserved			:4;
        uint8_t GestureType			:4;		//	0 -> Garbage Model
                                            //  1 -> Flick Gesture
                                            //	2 -> Circular Gesture
        uint8_t EdgeFlick			:1;		//	If "1" Edge Flick
        uint16_t Reserved2			:14;
        uint8_t GestureInProgress	:1;		//	If "1" Gesture recognition in progress
    } Bit;
} GestureInfo;

uint32_t LastTouch;
union TouchInfo {
    uint32_t Touch;
    uint8_t  TouchArray[4];
    struct {
        uint8_t  Byte_0;
        uint8_t  Byte_1;
        uint8_t  Byte_2;
        uint8_t  Byte_3;
    } TouchByte;
    struct {
        uint8_t TouchSouth			:1;	//	Bit 00 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TouchWest			:1;	//	Bit 01 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TouchNorth			:1;	//	Bit 02 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TouchEast			:1;	//	Bit 03 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TouchCentre			:1;	//	Bit 04 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TapSouth			:1;	//	Bit 05 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TapWest				:1;	//	Bit 06 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TapNorth			:1;	//	Bit 07 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TapEast				:1;	//	Bit 08 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t TapCentre			:1;	//	Bit 09 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t DoubleTapSouth		:1;	//	Bit 10 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t DoubleTapWest		:1;	//	Bit 11 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t DoubleTapNorth		:1;	//	Bit 12 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t DoubleTapEast		:1;	//	Bit 13 Touch Info Sensor_Data_Output (ID = 0x91)
        uint8_t DoubleTapCentre		:1;	//	Bit 14 Touch Info Sensor_Data_Output (ID = 0x91)
        uint32_t FreeBit			:17;
    } Bit;	
} TouchInfo;
//----------------------------------------

uint16_t Previous_x_pos;
uint16_t Previous_y_pos;
uint16_t Previous_z_pos;



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void ACTUAL_NRF_LOG_RAW_HEXDUMP_INFO(uint8_t * data, int len){
    if(len > 80){
        for(int i = 0; i < len; i += 80){
            NRF_LOG_RAW_HEXDUMP_INFO((data + i), 80);
        }
        int remaining = (len % 80);
        if(remaining != 0)
            NRF_LOG_RAW_HEXDUMP_INFO((data + (len - remaining)), remaining);
    } else {
        NRF_LOG_RAW_HEXDUMP_INFO(data, len);
    }
    NRF_LOG_FLUSH();
}

void mgc3130_get_data (uint8_t amount) {
	//int  Counter = 0;
    char log_buffer[150];
    memset(m_sample, 0, amount);

    m_xfer_done = false;

    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, MGC3130_ADDR, &m_sample[0], amount);
    APP_ERROR_CHECK(err_code);
	do {
        __WFE();
    } while (m_xfer_done == false);

    ACTUAL_NRF_LOG_RAW_HEXDUMP_INFO(m_sample, amount);
    
	switch (m_sample[3])
	{
		case ID_FW_VERSION:		
			if (m_sample[4] == 0xAA) {
				// Valid Gestic Library available

                memcpy(log_buffer, m_sample + 10, sizeof(char) * 128);
			}
			break;
			
		case ID_DATA_OUTPUT:		
			// ----------------------------------------
			// Save Data into internal array
			for (int i = 0; i < 4; i++) {
				GestureInfo.GestArray[i] = m_sample[i + 10];
				TouchInfo.TouchArray[i]  = m_sample[i + 14];
			}		
			GestureInfo.Gesture &= MASK_GESTURE_RAW;
			TouchInfo.Touch     &= MASK_TOUCH_RAW;
			AirWheelInfo = m_sample[18];
			for (int i = 0; i < 6; i++) {
				xyzPosition.xyzArray[i] = m_sample[i + 20];
			}
			// ----------------------------------------
            // NRF_LOG_INFO("DATA RECIEVED");
			break;

        case 0x15:
            // NRF_LOG_INFO("SYSTEM STATUS");
            break;

        case 0x06:
            // NRF_LOG_INFO("REQUEST MESSAGE");
            break;

        case 0xA2:
            // NRF_LOG_INFO("SET RUNTIME PARAMETER");
            break;

		default:
            // NRF_LOG_INFO("NO DATA RECIEVED");
			break;
	}
}

void mgc3130_reset_board(void){
    nrf_gpio_pin_clear(MGC_3130_RS_PIN);
    nrf_delay_ms(250);
    nrf_gpio_pin_set(MGC_3130_RS_PIN);
    nrf_delay_ms(250);
}

/**
 * @brief MGC3130 initialization.
 */
void mgc3130_init (void)
{
    nrf_gpio_cfg(MGC_3130_RS_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg_input(MGC_3130_TS_PIN, NRF_GPIO_PIN_NOPULL);
    mgc3130_reset_board();
    mgc3130_get_data(0x86);
    NRF_LOG_INFO("MGC3130 Ready.");
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            mgc3130_release_ts_line();
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static int idle_state_handle(void)
{
    // app_sched_execute();
    if(mgc3130_get_ts_line_status() == 0){
        mgc3130_get_data(0x1A);
        nrf_delay_ms(10);
        if(m_sample[0] == 0)
            return 1;
        return 0;
    }
    return 0;
    // if (NRF_LOG_PROCESS() == false)
    // {
    //     nrf_pwr_mgmt_run();
    // }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    int error_count = 0;
    // Initialize.
    log_init();
    // timers_init();
    // buttons_leds_init(&erase_bonds);
    // power_management_init();
    // ble_stack_init();
    // scheduler_init();
    // gap_params_init();
    // gatt_init();
    // advertising_init();
    // services_init();
    // sensor_simulator_init();
    // conn_params_init();
    // peer_manager_init();

    twi_init();
    mgc3130_init();

    // Start execution.
    NRF_LOG_INFO("HID Mouse example started.");

    // Enter main loop.
    for (;;)
    {
        error_count += idle_state_handle();
        if(error_count > 3){
            mgc3130_init();
            error_count = 0;
        }
    }
}


/**
 * @}
 */

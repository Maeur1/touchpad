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

//-----------------------------------------------------------------------
//	GESTURE INPUT CODE MASK
#define NO_GESTURE						0x00
#define GESTURE_GARBAGE					0x01
#define GESTURE_WEST_EAST				0x02
#define GESTURE_EAST_WEST				0x03
#define GESTURE_SOUTH_NORTH				0x04
#define GESTURE_NORTH_SOUTH				0x05
#define GESTURE_CLOCK_WISE				0x06
#define GESTURE_COUNTER_CLOCK_WISE		0x07
#define GESTURE_WAVE_X					0x08
#define GESTURE_WAVE_Y					0x09
#define GESTURE_HOLD					0x40
#define GESTURE_PRESENCE				0x49
#define GESTURE_EDGE_WEST_EAST			0x41
#define GESTURE_EDGE_EAST_WEST			0x42
#define GESTURE_EDGE_SOUTH_NORTH		0x43
#define GESTURE_EDGE_NORTH_SOUTH		0x44
#define GESTURE_DOUBLE_WEST_EAST		0x45
#define GESTURE_DOUBLE_EAST_WEST		0x46
#define GESTURE_DOUBLE_SOUTH_NORTH		0x47
#define GESTURE_DOUBLE_NORTH_SOUTH		0x48
//-----------------------------------------------------------------------

//	TOUCH/GESTURE OUTPUT MASK
#define GESTURE_MASK_TOUCH_SOUTH			(uint64_t)(0x0000000000000001)
#define GESTURE_MASK_TOUCH_WEST				(uint64_t)(0x0000000000000002)
#define GESTURE_MASK_TOUCH_NORTH			(uint64_t)(0x0000000000000004)
#define GESTURE_MASK_TOUCH_EAST				(uint64_t)(0x0000000000000008)
#define GESTURE_MASK_TOUCH_CENTRE			(uint64_t)(0x0000000000000010)
#define GESTURE_MASK_TAP_SOUTH				(uint64_t)(0x0000000000000020)
#define GESTURE_MASK_TAP_WEST				(uint64_t)(0x0000000000000040)
#define GESTURE_MASK_TAP_NORTH				(uint64_t)(0x0000000000000080)
#define GESTURE_MASK_TAP_EAST				(uint64_t)(0x0000000000000100)
#define GESTURE_MASK_TAP_CENTRE				(uint64_t)(0x0000000000000200)
#define GESTURE_MASK_DOUBLE_TAP_SOUTH		(uint64_t)(0x0000000000000400)
#define GESTURE_MASK_DOUBLE_TAP_WEST		(uint64_t)(0x0000000000000800)
#define GESTURE_MASK_DOUBLE_TAP_NORTH		(uint64_t)(0x0000000000001000)
#define GESTURE_MASK_DOUBLE_TAP_EAST		(uint64_t)(0x0000000000002000)
#define GESTURE_MASK_DOUBLE_TAP_CENTRE		(uint64_t)(0x0000000000004000)
#define GESTURE_MASK_WEST_EAST				(uint64_t)(0x0000000000008000)
#define GESTURE_MASK_EAST_WEST				(uint64_t)(0x0000000000010000)
#define GESTURE_MASK_SOUTH_NORTH			(uint64_t)(0x0000000000020000)
#define GESTURE_MASK_NORTH_SOUTH			(uint64_t)(0x0000000000040000)
#define GESTURE_MASK_EDGE_WEST_EAST			(uint64_t)(0x0000000000080000)
#define GESTURE_MASK_EDGE_EAST_WEST			(uint64_t)(0x0000000000100000)
#define GESTURE_MASK_EDGE_SOUTH_NORTH		(uint64_t)(0x0000000000200000)
#define GESTURE_MASK_EDGE_NORTH_SOUTH		(uint64_t)(0x0000000000400000)
#define GESTURE_MASK_CLOCK_WISE				(uint64_t)(0x0000000000800000)
#define GESTURE_MASK_COUNTER_CLOCK_WISE		(uint64_t)(0x0000000001000000)
#define GESTURE_MASK_WAVE_X					(uint64_t)(0x0000000002000000)
#define GESTURE_MASK_WAVE_Y					(uint64_t)(0x0000000004000000)
#define GESTURE_MASK_HOLD					(uint64_t)(0x0000000008000000)
#define GESTURE_MASK_PRESENCE				(uint64_t)(0x0000000010000000)
#define GESTURE_MASK_DOUBLE_WEST_EAST		(uint64_t)(0x0000000020000000)
#define GESTURE_MASK_DOUBLE_EAST_WEST		(uint64_t)(0x0000000040000000)
#define GESTURE_MASK_DOUBLE_SOUTH_NORTH		(uint64_t)(0x0000000080000000)
#define GESTURE_MASK_DOUBLE_NORTH_SOUTH		(uint64_t)(0x0000000100000000)
//-----------------------------------------------------------------------

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


APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
BLE_HIDS_DEF(m_hids,                                                                /**< HID service instance. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REP_BUTTONS_LEN,
             INPUT_REP_MOVEMENT_LEN,
             INPUT_REP_MEDIA_PLAYER_LEN);
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static bool              m_in_boot_mode = false;                                    /**< Current protocol mode. */
static uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                                 /**< Device reference handle to the current bonded central. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static pm_peer_id_t      m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];       /**< List of peers currently in the whitelist. */
static uint32_t          m_whitelist_peer_cnt;                                      /**< Number of peers currently in the whitelist. */
static ble_uuid_t        m_adv_uuids[] =                                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
};

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

//----------------------------------------
//	Gesture decoded. Global variable
union GestureOutput {
    uint64_t Gesture;
    uint8_t  GestArray[8];
    struct {
        uint8_t  Byte_0;
        uint8_t  Byte_1;
        uint8_t  Byte_2;
        uint8_t  Byte_3;
        uint8_t  Byte_4;
        uint8_t  Byte_5;
        uint8_t  Byte_6;
        uint8_t  Byte_7;
    } GestureByte;			
    struct {
        uint8_t TouchSouth				:1;		//	GESTURE_TOUCH_SOUTH			0x0000000000000001
        uint8_t TouchWest				:1;		//	GESTURE_TOUCH_WEST			0x0000000000000002
        uint8_t TouchNorth				:1;		//	GESTURE_TOUCH_NORTH			0x0000000000000004
        uint8_t TouchEast				:1;		//	GESTURE_TOUCH_EAST			0x0000000000000008
        uint8_t TouchCentre				:1;		//	GESTURE_TOUCH_CENTRE		0x0000000000000010
        uint8_t TapSouth				:1;		//	GESTURE_TAP_SOUTH			0x0000000000000020
        uint8_t TapWest					:1;		//	GESTURE_TAP_WEST			0x0000000000000040
        uint8_t TapNorth				:1;		//	GESTURE_TAP_NORTH			0x0000000000000080
        uint8_t TapEast					:1;		//	GESTURE_TAP_EAST			0x0000000000000100
        uint8_t TapCentre				:1;		//	GESTURE_TAP_CENTRE			0x0000000000000200
        uint8_t DoubleTapSouth			:1;		//	GESTURE_DOUBLE_TAP_SOUTH	0x0000000000000400
        uint8_t DoubleTapWest			:1;		//	GESTURE_DOUBLE_TAP_WEST		0x0000000000000800
        uint8_t DoubleTapNorth			:1;		//	GESTURE_DOUBLE_TAP_NORTH	0x0000000000001000
        uint8_t DoubleTapEast			:1;		//	GESTURE_DOUBLE_TAP_EAST		0x0000000000002000
        uint8_t DoubleTapCentre			:1;		//	GESTURE_DOUBLE_TAP_CENTRE	0x0000000000004000
        uint8_t GestWestEast			:1;		//	GESTURE_WEST_EAST			0x0000000000008000
        uint8_t GestEastWest			:1;		//	GESTURE_EAST_WEST			0x0000000000010000
        uint8_t GestSouthNorth			:1;		//	GESTURE_SOUTH_NORTH			0x0000000000020000
        uint8_t GestNorthSouth			:1;		//	GESTURE_NORTH_SOUTH			0x0000000000040000
        uint8_t EdgeGestWestEast		:1;		//	GESTURE_EDGE_WEST_EAST		0x0000000000080000
        uint8_t EdgeGestEastWest		:1;		//	GESTURE_EDGE_EAST_WEST		0x0000000000100000
        uint8_t EdgeGestSouthNorth		:1;		//	GESTURE_EDGE_SOUTH_NORTH	0x0000000000200000
        uint8_t EdgeGestNorthSouth		:1;		//	GESTURE_EDGE_NORTH_SOUTH	0x0000000000400000
        uint8_t GestClockWise			:1;		//	GESTURE_CLOCK_WISE			0x0000000000800000
        uint8_t GestCounterClockWise	:1;		//	GESTURE_COUNTER_CLOCK_WISE	0x0000000001000000
        uint8_t GestWaveX				:1;		//	GESTURE_WAVE_X				0x0000000002000000
        uint8_t GestWaveY				:1;		//	GESTURE_WAVE_Y				0x0000000004000000
        uint8_t GestHold				:1;		//	GESTURE_HOLD				0x0000000008000000
        uint8_t GestPresence			:1;		//	GESTURE_PRESENCE			0x0000000010000000
        uint8_t DoubleGestWestEast		:1;		//	GESTURE_DOUBLE_WEST_EAST	0x0000000020000000
        uint8_t DoubleGestEastWest		:1;		//	GESTURE_DOUBLE_EAST_WEST	0x0000000040000000
        uint8_t DoubleSouthNorth		:1;		//	GESTURE_DOUBLE_SOUTH_NORTH	0x0000000080000000
        uint8_t DoubleGestNorthSouth	:1;		//	GESTURE_DOUBLE_NORTH_SOUTH	0x0000000100000000
        uint32_t FreeBit					:31;	//	FREE BIT
    } Bit;		
} GestureOutput;


static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);


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


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t ret;

        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(ret);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (ret != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(ret);
        }

        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);

            m_peer_id = p_evt->peer_id;
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
                // Note: You should check on what kind of white list policy your application should use.

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    // The whitelist has been modified, update it in the Peer Manager.
                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);

                    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (err_code != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                }
            }
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
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


/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                err_code;
    ble_hids_init_t           hids_init_obj;
    ble_hids_inp_rep_init_t * p_input_report;
    uint8_t                   hid_info_flags;

    static ble_hids_inp_rep_init_t inp_rep_array[INPUT_REPORT_COUNT];
    static uint8_t rep_map_data[] =
    {
        0x05, 0x01, // Usage Page (Generic Desktop)
        0x09, 0x02, // Usage (Mouse)

        0xA1, 0x01, // Collection (Application)

        // Report ID 1: Mouse buttons + scroll/pan
        0x85, 0x01,       // Report Id 1
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x95, 0x05,       // Report Count (3)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x09,       // Usage Page (Buttons)
        0x19, 0x01,       // Usage Minimum (01)
        0x29, 0x05,       // Usage Maximum (05)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x01,       // Logical Maximum (1)
        0x81, 0x02,       // Input (Data, Variable, Absolute)
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x81, 0x01,       // Input (Constant) for padding
        0x75, 0x08,       // Report Size (8)
        0x95, 0x01,       // Report Count (1)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x38,       // Usage (Wheel)
        0x15, 0x81,       // Logical Minimum (-127)
        0x25, 0x7F,       // Logical Maximum (127)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0x05, 0x0C,       // Usage Page (Consumer)
        0x0A, 0x38, 0x02, // Usage (AC Pan)
        0x95, 0x01,       // Report Count (1)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0,             // End Collection (Physical)

        // Report ID 2: Mouse motion
        0x85, 0x02,       // Report Id 2
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x75, 0x0C,       // Report Size (12)
        0x95, 0x02,       // Report Count (2)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x30,       // Usage (X)
        0x09, 0x31,       // Usage (Y)
        0x16, 0x01, 0xF8, // Logical maximum (2047)
        0x26, 0xFF, 0x07, // Logical minimum (-2047)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0xC0,             // End Collection (Physical)
        0xC0,             // End Collection (Application)

        // Report ID 3: Advanced buttons
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0x03,       // Report Id (3)
        0x15, 0x00,       // Logical minimum (0)
        0x25, 0x01,       // Logical maximum (1)
        0x75, 0x01,       // Report Size (1)
        0x95, 0x01,       // Report Count (1)

        0x09, 0xCD,       // Usage (Play/Pause)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB5,       // Usage (Scan Next Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB6,       // Usage (Scan Previous Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

        0x09, 0xEA,       // Usage (Volume Down)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xE9,       // Usage (Volume Up)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x25, 0x02, // Usage (AC Forward)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x24, 0x02, // Usage (AC Back)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0              // End Collection
    };

    memset(inp_rep_array, 0, sizeof(inp_rep_array));
    // Initialize HID Service.
    p_input_report                      = &inp_rep_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = true;
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    qwr_init();
    dis_init();
    bas_init();
    hids_init();
}


/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {

                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.

            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled               = true;
    init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for sending a Mouse Movement.
 *
 * @param[in]   x_delta   Horizontal movement.
 * @param[in]   y_delta   Vertical movement.
 */
// static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
// {
//     ret_code_t err_code;

//     if (m_in_boot_mode)
//     {
//         x_delta = MIN(x_delta, 0x00ff);
//         y_delta = MIN(y_delta, 0x00ff);

//         err_code = ble_hids_boot_mouse_inp_rep_send(&m_hids,
//                                                     0x00,
//                                                     (int8_t)x_delta,
//                                                     (int8_t)y_delta,
//                                                     0,
//                                                     NULL,
//                                                     m_conn_handle);
//     }
//     else
//     {
//         uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

//         APP_ERROR_CHECK_BOOL(INPUT_REP_MOVEMENT_LEN == 3);

//         x_delta = MIN(x_delta, 0x0fff);
//         y_delta = MIN(y_delta, 0x0fff);

//         buffer[0] = x_delta & 0x00ff;
//         buffer[1] = ((y_delta & 0x000f) << 4) | ((x_delta & 0x0f00) >> 8);
//         buffer[2] = (y_delta & 0x0ff0) >> 4;

//         err_code = ble_hids_inp_rep_send(&m_hids,
//                                          INPUT_REP_MOVEMENT_INDEX,
//                                          INPUT_REP_MOVEMENT_LEN,
//                                          buffer,
//                                          m_conn_handle);
//     }

//     if ((err_code != NRF_SUCCESS) &&
//         (err_code != NRF_ERROR_INVALID_STATE) &&
//         (err_code != NRF_ERROR_RESOURCES) &&
//         (err_code != NRF_ERROR_BUSY) &&
//         (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//        )
//     {
//         APP_ERROR_HANDLER(err_code);
//     }
// }


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
// static void bsp_event_handler(bsp_event_t event)
// {
//     ret_code_t err_code;

//     switch (event)
//     {
//         case BSP_EVENT_SLEEP:
//             sleep_mode_enter();
//             break;

//         case BSP_EVENT_DISCONNECT:
//             err_code = sd_ble_gap_disconnect(m_conn_handle,
//                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//             if (err_code != NRF_ERROR_INVALID_STATE)
//             {
//                 APP_ERROR_CHECK(err_code);
//             }
//             break;

//         case BSP_EVENT_WHITELIST_OFF:
//             if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//             {
//                 err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                 if (err_code != NRF_ERROR_INVALID_STATE)
//                 {
//                     APP_ERROR_CHECK(err_code);
//                 }
//             }
//             break;

//         case BSP_EVENT_KEY_0:
//             if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
//             {
//                 mouse_movement_send(-MOVEMENT_SPEED, 0);
//             }
//             break;

//         case BSP_EVENT_KEY_1:
//             if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
//             {
//                 mouse_movement_send(0, -MOVEMENT_SPEED);
//             }
//             break;

//         case BSP_EVENT_KEY_2:
//             if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
//             {
//                 mouse_movement_send(MOVEMENT_SPEED, 0);
//             }
//             break;

//         case BSP_EVENT_KEY_3:
//             if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
//             {
//                 mouse_movement_send(0, MOVEMENT_SPEED);
//             }
//             break;

//         default:
//             break;
//     }
// }


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
// static void buttons_leds_init(bool * p_erase_bonds)
// {
//     ret_code_t err_code;
//     bsp_event_t startup_event;

//     err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);

//     APP_ERROR_CHECK(err_code);

//     err_code = bsp_btn_ble_init(NULL, &startup_event);
//     APP_ERROR_CHECK(err_code);

//     *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
// }


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void mgc3130_decode_gesture(void) {
    uint32_t Mask = 0x00000001;
    if (((TouchInfo.Touch ^ LastTouch) > 0) || ((GestureInfo.Gesture ^ LastGesture) > 0) ) {
        GestureOutput.Gesture = 0;
        if ((TouchInfo.Touch ^ LastTouch) > 0) {
            LastTouch = TouchInfo.Touch;
            for (int i = 0; i < 15; i++) {
                if ((TouchInfo.Touch & Mask) > 0) {
                    GestureOutput.Gesture |= Mask; 
                }
                Mask = Mask << 1;
            }
        } else if ((GestureInfo.Gesture ^ LastGesture) > 0) {
            LastGesture = GestureInfo.Gesture;
            switch (GestureInfo.Bit.GestureCode)
            {
                case NO_GESTURE:
                case GESTURE_GARBAGE:
                    break;
                case GESTURE_EDGE_EAST_WEST:
                    GestureOutput.Gesture |= GESTURE_MASK_EDGE_EAST_WEST;
                    break;
                case GESTURE_EDGE_WEST_EAST:
                    GestureOutput.Gesture |= GESTURE_MASK_EDGE_WEST_EAST;
                    break;
                case GESTURE_EDGE_SOUTH_NORTH:
                    GestureOutput.Gesture |= GESTURE_MASK_EDGE_SOUTH_NORTH;
                    break;
                case GESTURE_EDGE_NORTH_SOUTH:
                    GestureOutput.Gesture |= GESTURE_MASK_EDGE_NORTH_SOUTH;
                    break;
                case GESTURE_WEST_EAST:
                    if (GestureInfo.Bit.EdgeFlick == 0) {
                        GestureOutput.Gesture |= GESTURE_MASK_WEST_EAST;
                    } else {
                        GestureOutput.Gesture |= GESTURE_MASK_EDGE_WEST_EAST;
                    }
                    break;	
                case GESTURE_EAST_WEST:
                    if (GestureInfo.Bit.EdgeFlick == 0) {
                        GestureOutput.Gesture |= GESTURE_MASK_EAST_WEST;
                    } else {
                        GestureOutput.Gesture |= GESTURE_MASK_EDGE_EAST_WEST;
                    }
                    break;
                case GESTURE_SOUTH_NORTH:
                    if (GestureInfo.Bit.EdgeFlick == 0) {
                        GestureOutput.Gesture |= GESTURE_MASK_SOUTH_NORTH;
                    } else {
                        GestureOutput.Gesture |= GESTURE_MASK_EDGE_SOUTH_NORTH;
                    }
                    break;
                case GESTURE_NORTH_SOUTH:
                    if (GestureInfo.Bit.EdgeFlick == 0) {
                        GestureOutput.Gesture |= GESTURE_MASK_NORTH_SOUTH;
                    } else {
                        GestureOutput.Gesture |= GESTURE_MASK_EDGE_NORTH_SOUTH;
                    }
                    break;
                case GESTURE_CLOCK_WISE:
                    GestureOutput.Gesture |= GESTURE_MASK_CLOCK_WISE;
                    break;
                case GESTURE_COUNTER_CLOCK_WISE:
                    GestureOutput.Gesture |= GESTURE_MASK_COUNTER_CLOCK_WISE;
                    break;
                case GESTURE_WAVE_X:
                    GestureOutput.Gesture |= GESTURE_MASK_WAVE_X;
                    break;
                case GESTURE_WAVE_Y:
                    GestureOutput.Gesture |= GESTURE_MASK_WAVE_Y;
                    break;
                case GESTURE_HOLD:
                    GestureOutput.Gesture |= GESTURE_MASK_HOLD;
                    break;
                case GESTURE_PRESENCE:
                    GestureOutput.Gesture |= GESTURE_MASK_PRESENCE;
                    break;
                case GESTURE_DOUBLE_WEST_EAST:
                    GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_WEST_EAST;
                    break;
                case GESTURE_DOUBLE_EAST_WEST:
                    GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_EAST_WEST;
                    break;
                case GESTURE_DOUBLE_SOUTH_NORTH:
                    GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_SOUTH_NORTH;
                    break;
                case GESTURE_DOUBLE_NORTH_SOUTH:
                    GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_NORTH_SOUTH;
                    break;
                default:
                    break;
            }
        }
        //	Remove not desired Touch or Gesture Info. See MASK_FILTER_GESTURE into MGC3130.h file for details
        GestureOutput.Gesture &= ~(MASK_FILTER_GESTURE);
    }
}

const char TouchSouth[]  				 = "Touch South";
const char TouchWest[]   				 = "Touch West";
const char TouchNorth[]  				 = "Touch North";
const char TouchEast[]   				 = "Touch East";
const char TouchCentre[] 				 = "Touch Centre";

const char TapSouth[]  					 = "Tap South";
const char TapWest[]   					 = "Tap West";
const char TapNorth[] 		 			 = "Tap North";
const char TapEast[]   					 = "Tap East";
const char TapCentre[] 					 = "Tap Centre";

const char DoubleTapSouth[]  			 = "Double Tap South";
const char DoubleTapWest[]   			 = "Double Tap West";
const char DoubleTapNorth[]  			 = "Double Tap North";
const char DoubleTapEast[]   			 = "Double Tap East";
const char DoubleTapCentre[] 			 = "Double Tap Centre";

const char GestureWestToEast[]   		 = "Gesture West to East";
const char GestureEastToWest[]   		 = "Gesture East to West";
const char GestureNorthToSouth[] 		 = "Gesture North to South";
const char GestureSouthToNorth[] 		 = "Gesture South to North";

const char GestureEdgeWestToEast[]   	 = "Gesture Edge West to East";
const char GestureEdgeEastToWest[]   	 = "Gesture Edge East to West";
const char GestureEdgeNorthToSouth[] 	 = "Gesture Edge North to South";
const char GestureEdgeSouthToNorth[] 	 = "Gesture Edge South to North";	

const char GestureClockWise[]        	 = "Gesture Clock Wise";	
const char GestureCounterClockWise[] 	 = "Gesture Counter Clock Wise";

const char GestureWaveX[] 			 	 = "Gesture Wave X";	
const char GestureWaveY[] 				 = "Gesture Wave Y";
const char GestureHold[] 			 	 = "Gesture Hold";	
const char GesturePresence[]			 = "Gesture Presence";

const char GestureDoubleWestToEast[]   	 = "Gesture Double West to East";
const char GestureDoubleEastToWest[]   	 = "Gesture Double East to West";
const char GestureDoubleNorthToSouth[] 	 = "Gesture Double North to South";
const char GestureDoubleSouthToNorth[] 	 = "Gesture Double South to North";

void print_mgc3130_gesture(void) {
    //----------------------------------------
    if (GestureOutput.Bit.TouchSouth > 0) {
        NRF_LOG_INFO("%s", TouchSouth);
    }
    if (GestureOutput.Bit.TouchWest > 0) {
        NRF_LOG_INFO("%s", TouchWest);
    }
    if (GestureOutput.Bit.TouchNorth > 0) {
        NRF_LOG_INFO("%s", TouchNorth);
    }
    if (GestureOutput.Bit.TouchEast > 0) {
        NRF_LOG_INFO("%s", TouchEast);
    }
    if (GestureOutput.Bit.TouchCentre > 0) {
        NRF_LOG_INFO("%s", TouchCentre);
    }	
    //----------------------------------------

    //----------------------------------------
    if (GestureOutput.Bit.TapSouth > 0) {
        NRF_LOG_INFO("%s", TapSouth);
    }
    if (GestureOutput.Bit.TapWest > 0) {
        NRF_LOG_INFO("%s", TapWest);
    }
    if (GestureOutput.Bit.TapNorth > 0) {
        NRF_LOG_INFO("%s", TapNorth);
    }
    if (GestureOutput.Bit.TapEast > 0) {
        NRF_LOG_INFO("%s", TapEast);
    }
    if (GestureOutput.Bit.TapCentre > 0) {
        NRF_LOG_INFO("%s", TapCentre);
    }
    //----------------------------------------
    
    //----------------------------------------
    if (GestureOutput.Bit.DoubleTapSouth > 0) {
        NRF_LOG_INFO("%s", DoubleTapSouth);
    }
    if (GestureOutput.Bit.DoubleTapWest > 0) {
        NRF_LOG_INFO("%s", DoubleTapWest);
    }
    if (GestureOutput.Bit.DoubleTapNorth > 0) {
        NRF_LOG_INFO("%s", DoubleTapNorth);
    }
    if (GestureOutput.Bit.DoubleTapEast > 0) {
        NRF_LOG_INFO("%s", DoubleTapEast);
    }
    if (GestureOutput.Bit.DoubleTapCentre > 0) {
        NRF_LOG_INFO("%s", DoubleTapCentre);
    }
    //----------------------------------------

    //----------------------------------------	
    if (GestureOutput.Bit.GestWestEast > 0) {
        NRF_LOG_INFO("%s", GestureWestToEast);
    }
    if (GestureOutput.Bit.GestEastWest > 0) {
        NRF_LOG_INFO("%s", GestureEastToWest);
    }	
    if (GestureOutput.Bit.GestSouthNorth > 0) {
        NRF_LOG_INFO("%s", GestureSouthToNorth);
    }
    if (GestureOutput.Bit.GestNorthSouth > 0) {
        NRF_LOG_INFO("%s", GestureNorthToSouth);
    }
    if (GestureOutput.Bit.EdgeGestWestEast > 0) {
        NRF_LOG_INFO("%s", GestureEdgeWestToEast);
    }
    if (GestureOutput.Bit.EdgeGestEastWest > 0) {
        NRF_LOG_INFO("%s", GestureEdgeEastToWest);
    }
    if (GestureOutput.Bit.EdgeGestSouthNorth > 0) {
        NRF_LOG_INFO("%s", GestureEdgeNorthToSouth);
    }
    if (GestureOutput.Bit.EdgeGestNorthSouth > 0) {
        NRF_LOG_INFO("%s", GestureEdgeSouthToNorth);
    }
    if (GestureOutput.Bit.GestClockWise > 0) {
        NRF_LOG_INFO("%s", GestureClockWise);
    }
    if (GestureOutput.Bit.GestCounterClockWise > 0) {
        NRF_LOG_INFO("%s", GestureCounterClockWise);
    }
    //----------------------------------------	
    
    //----------------------------------------	
    if (GestureOutput.Bit.GestWaveX > 0) {
        NRF_LOG_INFO("%s", GestureWaveX);
    }		
    if (GestureOutput.Bit.GestWaveY > 0) {
        NRF_LOG_INFO("%s", GestureWaveY);
    }
    if (GestureOutput.Bit.GestHold > 0) {
        NRF_LOG_INFO("%s", GestureHold);
    }		
    if (GestureOutput.Bit.GestPresence > 0) {
        NRF_LOG_INFO("%s", GesturePresence);
    }	
    //----------------------------------------
    
    //----------------------------------------
    if (GestureOutput.Bit.DoubleGestWestEast > 0) {
        NRF_LOG_INFO("%s", GestureDoubleWestToEast);
    }		
    if (GestureOutput.Bit.DoubleGestEastWest > 0) {
        NRF_LOG_INFO("%s", GestureDoubleEastToWest);
    }	
    if (GestureOutput.Bit.DoubleSouthNorth > 0) {
        NRF_LOG_INFO("%s", GestureDoubleSouthToNorth);
    }	
    if (GestureOutput.Bit.DoubleGestNorthSouth > 0) {
        NRF_LOG_INFO("%s", GestureDoubleNorthToSouth);
    }	
    //----------------------------------------
    NRF_LOG_FLUSH();
}

void print_mgc3130_xyz(void) {
    if (Previous_x_pos != xyzPosition.xyzWord.x_pos) {
        Previous_x_pos = xyzPosition.xyzWord.x_pos;
    }
    if (Previous_y_pos != xyzPosition.xyzWord.y_pos) {
        Previous_y_pos = xyzPosition.xyzWord.y_pos;
    }
    if (Previous_z_pos != xyzPosition.xyzWord.z_pos) {
        Previous_z_pos = xyzPosition.xyzWord.z_pos;		
    }
    NRF_LOG_INFO("X: %d Y: %d Z: %d", xyzPosition.xyzWord.x_pos, xyzPosition.xyzWord.y_pos, xyzPosition.xyzWord.z_pos);
    NRF_LOG_FLUSH();
    nrf_gpio_pin_set(MGC_3130_LED0_PIN);
}

int mgc3130_get_ts_line_status (void) {
    if(nrf_gpio_pin_read(MGC_3130_TS_PIN) == 0){
        nrf_gpio_cfg(MGC_3130_TS_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_pin_clear(MGC_3130_TS_PIN);
        return 0;
    }
    return 1;
} 

void mgc3130_release_ts_line (void) {
    nrf_gpio_pin_set(MGC_3130_TS_PIN);
    nrf_gpio_cfg(MGC_3130_TS_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
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

    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, MGC3130_ADDR, m_sample, amount);
    APP_ERROR_CHECK(err_code);
	do {
        __WFE();
    } while (m_xfer_done == false);

    // ACTUAL_NRF_LOG_RAW_HEXDUMP_INFO(m_sample, amount);
    
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
    nrf_gpio_cfg(MGC_3130_TS_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg(MGC_3130_LED0_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(MGC_3130_LED1_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    mgc3130_reset_board();
    mgc3130_get_data(0x82);
    NRF_LOG_INFO("MGC3130 Ready.");
    nrf_delay_ms(200);
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
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
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
    app_sched_execute();
    if(mgc3130_get_ts_line_status() == 0){
        mgc3130_get_data(0x1A);
        if(m_sample[0] == 0)
            return 1;
        print_mgc3130_xyz();
        return 0;
    } else {
        nrf_gpio_pin_clear(MGC_3130_LED0_PIN);
    }
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
    return 0;
}

/**@brief Function for application main entry.
 */
int main(void)
{
    int error_count = 0;
    bool erase_bonds = false;

    // Initialize.
    log_init();
    timers_init();
    // buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();

    twi_init();
    mgc3130_init();

    // Start execution.
    NRF_LOG_INFO("HID Mouse example started.");
    timers_start();
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        error_count += idle_state_handle();
        if(error_count > 20){
            mgc3130_init();
            error_count = 0;
        }
    }
}
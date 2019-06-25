#ifndef _MGC3130_H
#define _MGC3130_H

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"

// #define PRINT_RAW_FW_INFO	//	Enable Print on Serial interface of RAW Firmware Info
// #define PRINT_RAW_DATA		//	Enable Print on Serial interface of RAW data
// #define PRINT_GESTURE_DATA	//	Enable Print on Serial interface of Gesture recognized
// #define PRINT_XYZ			//	Enable Print on serial interface of xyz coordinates

// #define DEBUG_MGC3130
#define MGC3130_ADDRESS					0x42
#define MGC3130_SCL                     27
#define MGC3130_SDA                     26
#define MGC3130_TS                      25
#define MGC3130_MCLR                    24
#define MGC3130_IRQ0                    23
#define MGC3130_IRQ1                    22
#define MGC3130_RESET_DELAY_MS          250
#define DEBUG_LED                       20
#define MIDDLE_VALUE					32767

//-----------------------------------------------------------------------
//	MGC3130 CMD ID
#define ID_DATA_OUTPUT		0x91
#define ID_FW_VERSION		0x83
//-----------------------------------------------------------------------

#define MASK_GESTURE_RAW	(uint32_t)(0x0001F0FF)	//	Filter mask to remove invalid data into gesture packet
#define MASK_TOUCH_RAW		(uint32_t)(0x00007FFF)	//	Filter mask to remove invalid data into touch packet

#define MASK_FILTER_GESTURE	(uint64_t)(0x0000000000000000)				//	To calculate exactly value of mask see below	
// B0000000000000000000000000000000000000000000000000000000000000000	//	Set bit to "1" to mask Gesture and convert binary data into hexadecimal data
//  ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//	|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||+------>		if "1" MASK Touch South
//	||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||+------->		if "1" MASK Touch West
//	|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||+-------->		if "1" MASK Touch North
//	||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||+--------->		if "1" MASK Touch East
//	|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||+---------->		if "1" MASK Touch Centre
//	||||||||||||||||||||||||||||||||||||||||||||||||||||||||||+----------->		if "1" MASK Tap South
//	|||||||||||||||||||||||||||||||||||||||||||||||||||||||||+------------>		if "1" MASK Tap West
//	||||||||||||||||||||||||||||||||||||||||||||||||||||||||+------------->		if "1" MASK Tap North
//	|||||||||||||||||||||||||||||||||||||||||||||||||||||||+-------------->		if "1" MASK Tap East
//	||||||||||||||||||||||||||||||||||||||||||||||||||||||+--------------->		if "1" MASK Tap Centre
//	|||||||||||||||||||||||||||||||||||||||||||||||||||||+---------------->		if "1" MASK Double Tap South
//	||||||||||||||||||||||||||||||||||||||||||||||||||||+----------------->		if "1" MASK Double Tap West
//	|||||||||||||||||||||||||||||||||||||||||||||||||||+------------------>		if "1" MASK Double Tap North
//	||||||||||||||||||||||||||||||||||||||||||||||||||+------------------->		if "1" MASK Double Tap East
//	|||||||||||||||||||||||||||||||||||||||||||||||||+-------------------->		if "1" MASK Double Tap Centre
//	||||||||||||||||||||||||||||||||||||||||||||||||+--------------------->		if "1" MASK Gesture West To East
//	|||||||||||||||||||||||||||||||||||||||||||||||+---------------------->		if "1" MASK Gesture East To West
//	||||||||||||||||||||||||||||||||||||||||||||||+----------------------->		if "1" MASK Gesture South To North
//	|||||||||||||||||||||||||||||||||||||||||||||+------------------------>		if "1" MASK Gesture North To South
//	||||||||||||||||||||||||||||||||||||||||||||+-------------------------> 	if "1" MASK Gesture Edge West To East
//	|||||||||||||||||||||||||||||||||||||||||||+-------------------------->		if "1" MASK Gesture Edge East To West
//	||||||||||||||||||||||||||||||||||||||||||+--------------------------->		if "1" MASK Gesture Edge South To North
//	|||||||||||||||||||||||||||||||||||||||||+---------------------------->		if "1" MASK Gesture Edge North To South
//	||||||||||||||||||||||||||||||||||||||||+----------------------------->		if "1" MASK Gesture Clock Wise
//	|||||||||||||||||||||||||||||||||||||||+------------------------------>		if "1" MASK Gesture Counter Clock Wise
//	||||||||||||||||||||||||||||||||||||||+------------------------------->		if "1" MASK Gesture Complete Rotation
//	|||||||||||||||||||||||||||||||||||||+-------------------------------->		if "1" MASK Gesture Wave X
//	||||||||||||||||||||||||||||||||||||+--------------------------------->		if "1" MASK Gesture Wave Y
//	|||||||||||||||||||||||||||||||||||+---------------------------------->		if "1" MASK Gesture Hold
//	||||||||||||||||||||||||||||||||||+----------------------------------->		if "1" MASK Gesture Presence
//	|||||||||||||||||||||||||||||||||+------------------------------------>		if "1" MASK Gesture Double West To East
//	||||||||||||||||||||||||||||||||+------------------------------------->		if "1" MASK Gesture Double East To West
//	|||||||||||||||||||||||||||||||+-------------------------------------->		if "1" MASK Gesture Double South To North
//	||||||||||||||||||||||||||||||+--------------------------------------->		if "1" MASK Gesture Double North To South
//	++++++++++++++++++++++++++++++---------------------------------------->		Free
									
//-----------------------------------------------------------------------
//	Use this MASK constant to create application code for gestic gesture decode 
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

#define MGC3130_FIRST_MESSAGE_SIZE		0x86
#define MGC3130_MESSAGE_SIZE			0x1A

//----------------------------------------


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
		uint32_t FreeBit				:31;	//	FREE BIT
	} Bit;		
} GestureOutput;
//----------------------------------------

//----------------------------------------
//	AirWheel Variable
uint8_t	AirWheelInfo;
//----------------------------------------

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
int32_t delta_x;
int32_t delta_y;

void mgc3130_init(const nrf_drv_twi_t* twi, void (*call)(int16_t, int16_t));
void mgc3130_reset(void);
void mgc3130_read_data(void);
void mgc3130_print_raw_firmware_info(void);
void mgc3130_print_xyz(void);

#endif
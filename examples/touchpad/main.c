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
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "MGC3130.h"
#include "DRV2605.h"

#define LOW                 0
#define HIGH                1


/* Indicates if operation on TWI has ended. */
static volatile bool mgc3130_xfer_done = false;
static volatile bool drv2605_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t mgc3130_twi = NRF_DRV_TWI_INSTANCE(0);
static const nrf_drv_twi_t drv2605_twi = NRF_DRV_TWI_INSTANCE(1);

/* MGCData */
static uint8_t mgc3130_data[255];
static bool first_read;
static int mgc3130_errors = 0;

// static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

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

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief TWI events handler.
 */
void mgc3130_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            mgc3130_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI events handler.
 */
void drv2605_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            drv2605_xfer_done = true;
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

    const nrf_drv_twi_config_t mgc3130_twi_config = {
       .scl                = MGC3130_TWI_SCL,
       .sda                = MGC3130_TWI_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    const nrf_drv_twi_config_t drv2605_twi_config = {
       .scl                = DRV2605_TWI_SCL,
       .sda                = DRV2605_TWI_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&mgc3130_twi, &mgc3130_twi_config, mgc3130_twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_init(&drv2605_twi, &drv2605_twi_config, drv2605_twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&mgc3130_twi);
    nrf_drv_twi_enable(&drv2605_twi);
}

uint8_t drv2605_read_register_8(uint8_t reg) {
    uint8_t rx_data;
    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(DRV2605_ADDR, &reg, 1, &rx_data, 1);
    uint32_t flags = 0;
    drv2605_xfer_done = false;
    nrf_drv_twi_xfer(&drv2605_twi, &xfer, flags); 
    do {
        __WFE();
    } while (drv2605_xfer_done == false);
    return rx_data;
}

void drv2605_write_register_8(uint8_t reg, uint8_t val) {
    uint8_t tx_data[] = {reg, val};
    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TX(DRV2605_ADDR, tx_data, 2);
    uint32_t flags = 0;
    drv2605_xfer_done = false;
    nrf_drv_twi_xfer(&drv2605_twi, &xfer, flags); 
    do {
        __WFE();
    } while (drv2605_xfer_done == false);
}

void drv2605_set_waveform (uint8_t slot, uint8_t w) {
    drv2605_write_register_8(DRV2605_REG_WAVESEQ1+slot, w);
}

void drv2605_select_library (uint8_t lib){
    drv2605_write_register_8(DRV2605_REG_LIBRARY, lib);
}

void drv2605_go (void){
    drv2605_write_register_8(DRV2605_REG_GO, 1);
}

void drv2605_set_mode(uint8_t mode) {
    drv2605_write_register_8(DRV2605_REG_MODE, mode);
}

void drv2605_motor_select(uint8_t val) {
    drv2605_write_register_8(DRV2605_REG_FEEDBACK, val);
}

void drv2605_auto_calibrate(){

    drv2605_write_register_8(DRV2605_REG_MODE, 0x07);
    drv2605_write_register_8(DRV2605_REG_FEEDBACK, (1 << 7) | (2 << 4) | (1 << 2));
    NRF_LOG_INFO("Register Set: %x Value: %x", DRV2605_REG_FEEDBACK, drv2605_read_register_8(DRV2605_REG_FEEDBACK)); 
    NRF_LOG_FLUSH();
    drv2605_write_register_8(DRV2605_REG_RATEDV, 200);
    NRF_LOG_INFO("Register Set: %x Value: %x", DRV2605_REG_RATEDV, drv2605_read_register_8(DRV2605_REG_RATEDV)); 
    NRF_LOG_FLUSH();
    drv2605_write_register_8(DRV2605_REG_CLAMPV, 200);
    NRF_LOG_INFO("Register Set: %x Value: %x", DRV2605_REG_CLAMPV, drv2605_read_register_8(DRV2605_REG_CLAMPV)); 
    NRF_LOG_FLUSH();
    drv2605_write_register_8(DRV2605_REG_CONTROL4, (3 << 4));
    NRF_LOG_INFO("Register Set: %x Value: %x", DRV2605_REG_CONTROL4, drv2605_read_register_8(DRV2605_REG_CONTROL4)); 
    NRF_LOG_FLUSH();
    drv2605_write_register_8(DRV2605_REG_CONTROL1, 28 | (1 << 7));
    NRF_LOG_INFO("Register Set: %x Value: %x", DRV2605_REG_CONTROL1, drv2605_read_register_8(DRV2605_REG_CONTROL1)); 
    NRF_LOG_FLUSH();
    drv2605_write_register_8(DRV2605_REG_CONTROL2, (3 << 6) | (3 << 4) | (1 << 2) | (1 << 0));
    NRF_LOG_INFO("Register Set: %x Value: %x", DRV2605_REG_CONTROL2, drv2605_read_register_8(DRV2605_REG_CONTROL2)); 
    NRF_LOG_FLUSH();
    drv2605_go();
    NRF_LOG_INFO("Autocalibration Started.");
    NRF_LOG_FLUSH();
    while(drv2605_read_register_8(DRV2605_REG_GO)){}
    uint8_t id = drv2605_read_register_8(DRV2605_REG_STATUS) & (1 << 3);
    NRF_LOG_INFO("Autocalibration Finished, Status: %d", id);
    NRF_LOG_FLUSH();
}

void drv2605_init (void) {
    uint8_t id = drv2605_read_register_8(DRV2605_REG_STATUS);
    NRF_LOG_INFO("ID: %x", id); 
    NRF_LOG_FLUSH();
    drv2605_auto_calibrate();
    // drv2605_write_register_8(DRV2605_REG_RATEDV, 155);
    // id = drv2605_read_register_8(DRV2605_REG_RATEDV);
    // NRF_LOG_INFO("Voltage Value: %d", id); 

    // drv2605_write_register_8(DRV2605_REG_MODE, 0x00); // out of standby

    // drv2605_write_register_8(DRV2605_REG_RTPIN, 0x00); // no real-time-playback

    // drv2605_write_register_8(DRV2605_REG_WAVESEQ1, 1); // strong click
    // drv2605_write_register_8(DRV2605_REG_WAVESEQ2, 0);

    // drv2605_write_register_8(DRV2605_REG_OVERDRIVE, 0); // no overdrive

    // drv2605_write_register_8(DRV2605_REG_SUSTAINPOS, 0);
    // drv2605_write_register_8(DRV2605_REG_SUSTAINNEG, 0);
    // drv2605_write_register_8(DRV2605_REG_BREAK, 0);
    // drv2605_write_register_8(DRV2605_REG_AUDIOMAX, 0x64);

    // // ERM open loop

    // turn off N_ERM_LRA
    drv2605_write_register_8(DRV2605_REG_FEEDBACK, drv2605_read_register_8(DRV2605_REG_FEEDBACK) | 0x80);
    // turn on LRA_OPEN_LOOP
    drv2605_write_register_8(DRV2605_REG_CONTROL3, drv2605_read_register_8(DRV2605_REG_CONTROL3) | 0x01);
    
}

static int get_ts_line_status (void) {
    if(nrf_gpio_pin_read(MGC3130_TS_PIN) == 0) {
        nrf_gpio_cfg_output(MGC3130_TS_PIN);
        nrf_gpio_pin_clear(MGC3130_TS_PIN);
        return TRUE;
    }
    return FALSE;
}

static void mgc3130_get_event (void) {
    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_RX(MGC3130_ADDR, mgc3130_data, ((first_read) ? MGC3130_FIRST_MESSAGE_SIZE : MGC3130_MESSAGE_SIZE));
    uint32_t flags = 0;
    mgc3130_xfer_done = false;
    ret_code_t ret = nrf_drv_twi_xfer(&mgc3130_twi, &xfer, flags); 
    do {
        __WFE();
    } while (mgc3130_xfer_done == false);
    APP_ERROR_CHECK(ret);
    // ACTUAL_NRF_LOG_RAW_HEXDUMP_INFO(mgc3130_data, ((first_read) ? MGC3130_FIRST_MESSAGE_SIZE : MGC3130_MESSAGE_SIZE));
    if(mgc3130_data[0] == 0)
        mgc3130_errors++;
    if(first_read)
        first_read = false;
}

static void mgc3130_release_ts_line (void) {
    nrf_gpio_pin_set(MGC3130_TS_PIN);
    nrf_gpio_cfg_input(MGC3130_TS_PIN, NRF_GPIO_PIN_PULLUP);
}

static void mgc3130_init (void) {
    nrf_gpio_cfg_output(MGC3130_LED0_PIN);
    nrf_gpio_pin_clear(MGC3130_LED0_PIN);
    nrf_gpio_cfg_output(MGC3130_LED1_PIN);
    nrf_gpio_pin_clear(MGC3130_LED1_PIN);
    nrf_gpio_cfg_input(MGC3130_TS_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(MGC3130_RS_PIN);
    nrf_gpio_pin_clear(MGC3130_RS_PIN);
    nrf_delay_ms(40);
    nrf_gpio_pin_set(MGC3130_RS_PIN);
    nrf_delay_ms(40);
    mgc3130_get_event();
    nrf_delay_ms(163);
    NRF_LOG_INFO("MGC3130 Init Finished");
}

static void mgc3130_reset (void) {
    first_read = true;
    mgc3130_init();
}

static void mgc3130_handle (void) {
    if(get_ts_line_status() == 0) {
        mgc3130_get_event();
        mgc3130_release_ts_line();
        nrf_gpio_pin_set(MGC3130_LED1_PIN);
    } else {
        nrf_gpio_pin_clear(MGC3130_LED1_PIN);
    }
    if(mgc3130_errors > 24) {
        mgc3130_reset();
        mgc3130_errors = 0;
    }
    nrf_delay_us(200);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // bool erase_bonds = false;
    first_read = true;

    // Initialize.
    log_init();

    twi_init();
    mgc3130_init();
    // drv2605_init();    
    // drv2605_select_library(1);

    // drv2605_set_waveform(5, 0);
    // for(int i = 0; i < 117; i++) {
    //     drv2605_set_waveform(0, i);
    //     drv2605_set_waveform(1, i);
    //     drv2605_set_waveform(2, i);
    //     drv2605_set_waveform(3, i);
    //     drv2605_set_waveform(4, i);
    //     drv2605_go();
    //     NRF_LOG_INFO("Effect: %d", i);
    //     NRF_LOG_FLUSH();
    //     while(drv2605_read_register_8(DRV2605_REG_GO)){}
    // }

    // Start execution.
    NRF_LOG_INFO("HID Mouse example started.");
    NRF_LOG_FLUSH();

    // Enter main loop.
    for (;;)
    {
        mgc3130_handle();        
    }
}
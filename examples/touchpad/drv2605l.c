#include "drv2605l.h"

static const nrf_drv_twi_t* m_twi;
/**************************************************************************/
/*!
  @brief Setup HW using a specified Wire
  @param theWire Pointer to a TwoWire object, defaults to &Wire
  @return Return value from init()
*/
/**************************************************************************/
bool drv2605l_begin(const nrf_drv_twi_t* twi) {
  m_twi = twi;
  return drv2605l_init();
}

/**************************************************************************/
/*!
  @brief  Setup the HW
  @return Always true
*/
/**************************************************************************/
bool drv2605l_init() {
  uint8_t id = drv2605l_readRegister8(DRV2605_REG_STATUS);
//   NRF_LOG_INFO("Status 0x%x", id);

  drv2605l_writeRegister8(DRV2605_REG_MODE, 0x00); // out of standby

  drv2605l_writeRegister8(DRV2605_REG_RTPIN, 0x00); // no real-time-playback

  drv2605l_writeRegister8(DRV2605_REG_WAVESEQ1, 1); // strong click
  drv2605l_writeRegister8(DRV2605_REG_WAVESEQ2, 0); // end sequence

  drv2605l_writeRegister8(DRV2605_REG_OVERDRIVE, 0); // no overdrive

  drv2605l_writeRegister8(DRV2605_REG_SUSTAINPOS, 0);
  drv2605l_writeRegister8(DRV2605_REG_SUSTAINNEG, 0);
  drv2605l_writeRegister8(DRV2605_REG_BREAK, 0);
  drv2605l_writeRegister8(DRV2605_REG_AUDIOMAX, 0x64);

  // ERM open loop

  // turn off N_ERM_LRA
  drv2605l_writeRegister8(DRV2605_REG_FEEDBACK, drv2605l_readRegister8(DRV2605_REG_FEEDBACK) & 0x7F);
  // turn on ERM_OPEN_LOOP
  drv2605l_writeRegister8(DRV2605_REG_CONTROL3, drv2605l_readRegister8(DRV2605_REG_CONTROL3) | 0x20);

  return true;
}

/**************************************************************************/
/*!
  @brief Select the haptic waveform to use.
  @param slot The waveform slot to set, from 0 to 7
  @param w The waveform sequence value, refers to an index in the ROM library.

    Playback starts at slot 0 and continues through to slot 7, stopping if it encounters
    a value of 0. A list of available waveforms can be found in section 11.2
    of the datasheet: http://www.adafruit.com/datasheets/DRV2605.pdf
*/
/**************************************************************************/
void drv2605l_setWaveform(uint8_t slot, uint8_t w) {
  drv2605l_writeRegister8(DRV2605_REG_WAVESEQ1+slot, w);
}

/**************************************************************************/
/*!
  @brief Select the waveform library to use.
  @param lib Library to use, 0 = Empty, 1-5 are ERM, 6 is LRA.

    See section 7.6.4 in the datasheet for more details: http://www.adafruit.com/datasheets/DRV2605.pdf
*/
/**************************************************************************/
void drv2605l_selectLibrary(uint8_t lib) {
  drv2605l_writeRegister8(DRV2605_REG_LIBRARY, lib);
}

/**************************************************************************/
/*!
  @brief Start playback of the waveforms (start moving!).
*/
/**************************************************************************/
void drv2605l_go() {
  drv2605l_writeRegister8(DRV2605_REG_GO, 1);
}

/**************************************************************************/
/*!
  @brief Stop playback.
*/
/**************************************************************************/
void drv2605l_stop() {
  drv2605l_writeRegister8(DRV2605_REG_GO, 0);
}

/**************************************************************************/
/*!
  @brief Set the device mode.
  @param mode Mode value, see datasheet section 7.6.2: http://www.adafruit.com/datasheets/DRV2605.pdf

    0: Internal trigger, call go() to start playback\n
    1: External trigger, rising edge on IN pin starts playback\n
    2: External trigger, playback follows the state of IN pin\n
    3: PWM/analog input\n
    4: Audio\n
    5: Real-time playback\n
    6: Diagnostics\n
    7: Auto calibration
*/
/**************************************************************************/
void drv2605l_setMode(uint8_t mode) {
  drv2605l_writeRegister8(DRV2605_REG_MODE, mode);
}

/**************************************************************************/
/*!
  @brief Set the realtime value when in RTP mode, used to directly drive the haptic motor.
  @param rtp 8-bit drive value.
*/
/**************************************************************************/
void drv2605l_setRealtimeValue(uint8_t rtp) {
  drv2605l_writeRegister8(DRV2605_REG_RTPIN, rtp);
}

/**************************************************************************/
/*!
  @brief Read an 8-bit register.
  @param reg The register to read.
  @return 8-bit value of the register.
*/
/**************************************************************************/
uint8_t drv2605l_readRegister8(uint8_t reg) {
  ret_code_t err_code;
  uint8_t x;

  // use i2c
  err_code = nrf_drv_twi_tx(m_twi, DRV2605_ADDR, &reg, sizeof(reg), false);
  if(err_code != NRF_SUCCESS)
  {
    NRF_LOG_INFO("Error Code: %d.", err_code);
  }
  err_code = nrf_drv_twi_rx(m_twi, DRV2605_ADDR, &x, 1);
  if(err_code != NRF_SUCCESS)
  {
    NRF_LOG_INFO("Error Code: %d.", err_code);
  }

//   NRF_LOG_INFO("$%x: 0x%x", reg, x);

  return x;
}

/**************************************************************************/
/*!
  @brief Write an 8-bit register.
  @param reg The register to write.
  @param val The value to write.
*/
/**************************************************************************/
void drv2605l_writeRegister8(uint8_t reg, uint8_t val) {
  ret_code_t err_code;
  uint8_t data[] = {reg, val};
  // use i2c
  err_code = nrf_drv_twi_tx(m_twi, DRV2605_ADDR, data, sizeof(data), false);
  if(err_code != NRF_SUCCESS)
  {
    NRF_LOG_INFO("Error Code: %d.", err_code);
  }
}

/**************************************************************************/
/*!
  @brief Use ERM (Eccentric Rotating Mass) mode.
*/
/**************************************************************************/
void drv2605l_useERM () {
  drv2605l_writeRegister8(DRV2605_REG_FEEDBACK, drv2605l_readRegister8(DRV2605_REG_FEEDBACK) & 0x7F);
}

/**************************************************************************/
/*!
  @brief Use LRA (Linear Resonance Actuator) mode.
*/
/**************************************************************************/
void drv2605l_useLRA () {
  drv2605l_writeRegister8(DRV2605_REG_FEEDBACK, drv2605l_readRegister8(DRV2605_REG_FEEDBACK) | 0x80);
}
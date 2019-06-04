#include "mgc3130.h"

static const nrf_drv_twi_t* m_twi;
uint8_t data[MGC3130_FIRST_MESSAGE_SIZE];

void (*callback)(int16_t, int16_t);

//----------------------------------------
//   X-Y-Z Position
static union xyzPosition{
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
//----------------------------------------

bool FIRST_READ;

void mgc3130_reset(void)
{
    nrf_gpio_pin_clear(MGC3130_MCLR);
    nrf_delay_ms(MGC3130_RESET_DELAY_MS);
    nrf_gpio_pin_set(MGC3130_MCLR);
    nrf_delay_ms(MGC3130_RESET_DELAY_MS);
}

void mgc3130_print_raw_firmware_info(void)
{
    NRF_LOG_INFO("Whole bunch of firmware stuff which I cannot be bothered formatting");
}

void mgc3130_print_xyz(void)
{
    if (Previous_x_pos != xyzPosition.xyzWord.x_pos) {
        Previous_x_pos = xyzPosition.xyzWord.x_pos;
    }
    if (Previous_y_pos != xyzPosition.xyzWord.y_pos) {
        Previous_y_pos = xyzPosition.xyzWord.y_pos;
    }
    if (Previous_z_pos != xyzPosition.xyzWord.z_pos) {
        Previous_z_pos = xyzPosition.xyzWord.z_pos;		
    }
    if( xyzPosition.xyzWord.x_pos || xyzPosition.xyzWord.y_pos || xyzPosition.xyzWord.z_pos)
        NRF_LOG_INFO("X:%6u\tY:%6u\tZ:%6u\t", 
            xyzPosition.xyzWord.x_pos, 
            xyzPosition.xyzWord.y_pos, 
            xyzPosition.xyzWord.z_pos)
}

void _mgc3130_process_delta(void)
{
    delta_x = (xyzPosition.xyzWord.x_pos - MIDDLE_VALUE)/1000;
    delta_y = (xyzPosition.xyzWord.y_pos - MIDDLE_VALUE)/1000;
    callback(delta_x, -delta_y);
#ifdef DEBUG_MGC3130
    if(xyzPosition.xyzWord.x_pos || xyzPosition.xyzWord.y_pos || xyzPosition.xyzWord.z_pos)
        NRF_LOG_INFO("Delta X: %5d\tY:%5d", delta_x, delta_y);
#endif
}

void mgc3130_read_data(void)
{
    ret_code_t err_code;
    if(FIRST_READ)
    {
        FIRST_READ = false;
        err_code = nrf_drv_twi_rx(m_twi, MGC3130_ADDRESS, data, MGC3130_FIRST_MESSAGE_SIZE);
    } 
    else
    {
        err_code = nrf_drv_twi_rx(m_twi, MGC3130_ADDRESS, data, MGC3130_MESSAGE_SIZE);
    }    
    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error Code: %d.", err_code);
    }

    char c;

    switch (data[3])
	{
		case ID_FW_VERSION:		
#ifdef PRINT_RAW_FW_INFO	
	mgc3130_print_raw_firmware_info();
#endif
			if (data[4] == 0xAA) {
				// Valid Gestic Library available
				NRF_LOG_RAW_INFO("FW Version: ");
				for (int i = 0; i < 120; i++) {
					c = (char)(data[i + 10]);
					NRF_LOG_RAW_INFO(&c);
				}
				NRF_LOG_RAW_INFO("\n");
			}
			break;
			
		case ID_DATA_OUTPUT:		
			// ----------------------------------------
			// Save Data into internal array
			for (int i = 0; i < 4; i++) {
				GestureInfo.GestArray[i] = data[i + 10];
				TouchInfo.TouchArray[i]  = data[i + 14];
			}		
			GestureInfo.Gesture &= MASK_GESTURE_RAW;
			TouchInfo.Touch     &= MASK_TOUCH_RAW;
			AirWheelInfo = data[18];
			for (int i = 0; i < 6; i++) {
				xyzPosition.xyzArray[i] = data[i + 20];
			}
			// ----------------------------------------
			break;
						
		default:
			break;
	}
#ifdef PRINT_XYZ
    mgc3130_print_xyz();
#endif
    _mgc3130_process_delta();
}

void mgc3130_interrupt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    int read_val = nrf_gpio_pin_read(MGC3130_TS);
#ifdef DEBUG_MGC3130
    nrf_gpio_pin_write(DEBUG_LED, read_val);
#endif
    if(read_val == 0)
        mgc3130_read_data();
}

void mgc3130_interrupt_init(void)
{
    ret_code_t err_code;

    if(!nrf_drv_gpiote_is_init())
    {
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(MGC3130_TS, &in_config, mgc3130_interrupt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MGC3130_TS, true);
}

void mgc3130_init(const nrf_drv_twi_t* twi, void (*call)(int16_t, int16_t))
{
    callback = call;
    m_twi = twi;
    FIRST_READ = true;
    nrf_gpio_cfg_input(MGC3130_TS, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(MGC3130_MCLR);
    mgc3130_reset();
    mgc3130_read_data();
    mgc3130_interrupt_init();
#ifdef DEBUG_MGC3130
    nrf_gpio_cfg_output(DEBUG_LED);
    nrf_gpio_pin_set(DEBUG_LED);
#endif
}
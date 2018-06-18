#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "math.h"
#include "stdlib.h"
#include "arm_math.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

#define WAKE_INTERVAL_IN_MS     50
#define XT_PERIOD               32768
#define WAKE_INTERVAL           XT_PERIOD * WAKE_INTERVAL_IN_MS * 1e-3
#define LCD_WAKE_INTERVAL_IN_MS 500
#define LCD_XT_PERIOD           32768
#define LCD_WAKE_INTERVAL       LCD_XT_PERIOD * LCD_WAKE_INTERVAL_IN_MS * 1e-3
#define IOM_MODULE_I2C     			0 // This will have a side benefit of testing IOM4 in offset mode
#define IOM_MODULE_SPI     			1
#define IOS_ADDRESS             0x20
#define CMD_RESET   0x1E  			// ADC reset command
#define CMD_ADC_READ 0x00  			// ADC read command
#define CMD_ADC_CONV 0x40  			// ADC conversion command
#define CMD_ADC_D1   0x00  	  	// ADC D1 conversion
#define CMD_ADC_D2   0x10    		// ADC D2 conversion
#define CMD_ADC_256  0x00    		// ADC OSR=256
#define CMD_ADC_512  0x02    		// ADC OSR=512
#define CMD_ADC_1024 0x04    		// ADC OSR=1024
#define CMD_ADC_2048 0x06    		// ADC OSR=2048
#define CMD_ADC_4096 0x08    		// ADC OSR=4096
#define CMD_PROM_RD  0xA0  			// Prom read command 
#define MS5611_I2C_ADRESS 0x76	// MS5611 adress 
#define DRIVE_SLAVE_RESET_PIN		14
#define D_C_PIN				13
#define SPI_SCLK_PIN	8
#define MOSI_PIN			10
#define SCE_PIN				12
#define BUZZER_PIN 25
#define BUZZER_PWM_TIMER 0

uint32_t coeff[] = {0,0,0,0,0,0,0,0};
uint32_t data_pressure = 0;
int32_t data_temperature = 0;
float32_t xt_init[] = {0, 0};
arm_matrix_instance_f32 xt = {2, 1, (float32_t *)xt_init};
am_hal_rtc_time_t hal_time;
uint32_t    g_LastSecond = 0;
uint32_t    g_TestCount = 0;


void pressure_sensor_read(void);
void itm_start(void);
void pressure_sensor_init(void);

//*****************************************************************************
//
// Start up the ITM interface.
//
//*****************************************************************************
void
itm_start(void)
{
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Clear the terminal.
    //
    am_util_stdio_terminal_clear();
}
//*****************************************************************************
//
// I2C Master Configuration
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMI2cConfig_i2c =
{
    .ui32InterfaceMode = AM_HAL_IOM_I2CMODE,
    .ui32ClockFrequency = AM_HAL_IOM_1MHZ,
};
//*****************************************************************************
//
// Configure the IOM as I2C master.
//
//*****************************************************************************
static void
iom_set_up(void)
{
	
    //
    // Enable power to IOM.
    //
    am_hal_iom_pwrctrl_enable(IOM_MODULE_I2C);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_config(IOM_MODULE_I2C, &g_sIOMI2cConfig_i2c);

    //
    // Set pins high to prevent bus dips.
    //
    am_hal_gpio_out_bit_set(5);
    am_hal_gpio_out_bit_set(6);
	
		//
		// Configure I2C pins
		//
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCL | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0SDA | AM_HAL_GPIO_PULLUP);

    //
    // Turn on the IOM for this operation.
    //
    am_bsp_iom_enable(IOM_MODULE_I2C);
}

//*****************************************************************************
//
// Initialize MS5611
//
//*****************************************************************************
void
pressure_sensor_init(void)
{
	
		//
		// Send reset command
		//
		uint8_t cmd = CMD_RESET;
		uint32_t res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
	
		//
		// Check if reset was succesful
		//
		if (res != 0) {
				am_util_stdio_printf("res = %d; initialization not succesfull\n", res);
		}
		else {
			
				uint32_t receive_coeff[] = {0,0,0,0,0,0,0,0};
				
				//
				// Read coefficients from Sensor with the PROM command
				//
				for(int i = 0;i<8;i++) {
					cmd = CMD_PROM_RD | (i << 1);
					am_util_delay_ms(2);
					res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
					am_util_delay_ms(2);
					res = am_hal_iom_i2c_read(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)receive_coeff + i, 2, AM_HAL_IOM_RAW);
					coeff[i] = ((receive_coeff[i] & 0x0000FF00) >> 8) | ((receive_coeff[i] & 0x000000FF) << 8);
			}
		}
		
		am_util_delay_ms(50);

		for (int i = 0; i < 100; i++) {
				pressure_sensor_read();
		}
		*xt.pData = data_pressure;
		
		//am_util_stdio_printf("%d\n", data_pressure);
	}
//*****************************************************************************
//
// Get Sensor Data
//
//*****************************************************************************	
	
void
pressure_sensor_read(void)
{
		uint32_t receive_data_pressure = 0;
		uint32_t receive_data_temperature = 0;
		
		//
		// Set up conversion mode for OSR 4096 (pressure)
		//
		uint8_t cmd = CMD_ADC_CONV+CMD_ADC_4096;
		uint32_t res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		am_util_delay_ms(10);
		
		//
		// Send a read command
		//
		cmd = CMD_ADC_READ;
		res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		
		//
		// Read pressure data from sensor
		//
		res = am_hal_iom_i2c_read(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&receive_data_pressure, 3, AM_HAL_IOM_RAW);
		
		//
		// Set up conversion mode for OSR 4096 (temperature)
		//
		cmd = CMD_ADC_CONV+CMD_ADC_4096+CMD_ADC_D2;
		res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		am_util_delay_ms(10);
		
		//
		// Send a read command
		//
		cmd = CMD_ADC_READ;
		res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		
		//
		// Read temperature data from sensor
		//
		res = am_hal_iom_i2c_read(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&receive_data_temperature, 3, AM_HAL_IOM_RAW);
		
		//
		// Invert LSBs and MSBs
		//
		data_pressure = ((receive_data_pressure & 0x000000FF) << 16) | ((receive_data_pressure & 0x0000FF00)) | ((receive_data_pressure & 0x00FF0000) >> 16);
		data_temperature = ((receive_data_temperature & 0x000000FF) << 16) | ((receive_data_temperature & 0x0000FF00)) | ((receive_data_temperature & 0x00FF0000) >> 16);

		//
		// Calculate temperature and pressure values
		//
		int32_t dT =  data_temperature - (coeff[5] << 8);
		int32_t TEMP = (2000 + ((dT*coeff[6]) >> 23));
		
		int64_t OFF = (coeff[2] << 16) + ((coeff[4]*dT) >> 7);
		int64_t SENS = (coeff[1] << 15) + ((coeff[3]*dT) >> 8);
		int64_t P = (((data_pressure*SENS) >> 21) - OFF) >> 15;
		
		//
		// Store the data
		//
		data_pressure = (uint32_t)P;
		data_temperature = TEMP;
}
//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    itm_start();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Allow time for all printing to finish.
    //
    am_util_delay_ms(10);

    //
    // Set up the IOM
    //
    iom_set_up();
		
		//
    // Enable the LFRC for the RTC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Select LFRC for RTC clock source.
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);
		
		//
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();
		
		//
    // The RTC is initialized from an arbitrary date.
    //
    hal_time.ui32Hour = 0;
    hal_time.ui32Minute = 0;
    hal_time.ui32Second = 0;
    hal_time.ui32Hundredths = 0;
    hal_time.ui32Weekday = 0;
    hal_time.ui32DayOfMonth = 0;
    hal_time.ui32Month = 0;
    hal_time.ui32Year = 0;
    hal_time.ui32Century = 0;
		
		am_hal_rtc_time_set(&hal_time);
		
		//
    // Initialize the sensor and read the coefficients
    //
		pressure_sensor_init();
	
		//
    // Loop forever.
    //
		
		while(true) {
				pressure_sensor_read();
				am_hal_rtc_time_get(&hal_time);
				uint32_t time = hal_time.ui32Hundredths + 100*hal_time.ui32Second + 6000*hal_time.ui32Minute;
				am_util_stdio_printf("%d\t%d\n", data_pressure, time);
				am_util_delay_ms(50);
		}
		
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

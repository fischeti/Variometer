#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "inttypes.h"
#include "math.h"


//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

#define WAKE_INTERVAL_IN_MS     100
#define XT_PERIOD               32768
#define WAKE_INTERVAL           XT_PERIOD * WAKE_INTERVAL_IN_MS * 1e-3
#define LCD_WAKE_INTERVAL_IN_MS 1234
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
#define MS5611_I2C_ADRESS 0x77	// MS5611 adress 
#define DRIVE_SLAVE_RESET_PIN		14
#define D_C_PIN				13
#define SPI_SCLK_PIN	8
#define MOSI_PIN			10
#define SCE_PIN				12
#define BUZZER_PIN 25
#define BUZZER_PWM_TIMER 0


//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
uint32_t BUZZER_BUZZER_WAKE_INTERVAL_IN_MS = 1000;
uint32_t BUZZER_XT_PERIOD = 32768;
uint32_t BUZZER_WAKE_INTERVAL = 32768 * 1000 * 1e-3;
float buzzer_small_frequency = 1200;
uint32_t buzzer_ctimer_counter = 10, buzzer_ctimer_duty_cycle = 5;
uint32_t toggle_bit = 1;

//*****************************************************************************
//
// Data Storage
//
//*****************************************************************************

uint32_t old_speed_string_size = 0;
uint32_t data_pressure = 0;
uint32_t data_temperature = 0;
uint32_t coeff[] = {0,0,0,0,0,0,0,0};
int32_t pressure_and_temperature[] = {0,0};
float velocity_array[] = {0,0,0,0,0,0,0,0,0,0};
uint8_t velocity_array_counter = 0;
static const uint8_t ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};


//*****************************************************************************
//
// Kalman Initialization
//
//*****************************************************************************
float dt = WAKE_INTERVAL_IN_MS;
float xt[] = {0, 0};
float P[] = {158806, 107, 124, 0.157};
float A[] = {1, 100, 0, 1};
float H[] = {1, 0};
float R = 1000000;
float Q[] = {0.0025, 0.005, 0.005, 0.01};
float eye[] = {1, 0, 0, 1};

//*****************************************************************************
//
// Function Declarations
//
//*****************************************************************************
void stimer_init(void);
void am_stimer_cmpr0_isr(void);
void am_iomaster0_isr(void);
void itm_start(void);
static void iom_set_up(void);
void pressure_sensor_init(void);
void pressure_sensor_read(void);
void kalman_filter(int32_t data);
void display_init(void);
void LcdString(char *characters, uint8_t x, uint8_t y);
float calc_velocity(float x_new, float x_old, float temp);
void buzzer_change_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency);
void init_cTimer(void);
void am_ctimer_isr(void);
void am_stimer_cmpr1_isr(void);


//*****************************************************************************
//
// BuzzerChange
//
//*****************************************************************************
void
buzzer_change_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency)
{
	//
	//Clear old Timers.
	//
	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
	am_hal_ctimer_clear(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA);
	
	//
	//Calculate Waketime for big Frequency
	//
	
	BUZZER_BUZZER_WAKE_INTERVAL_IN_MS = 1000 * (1.0 / desired_big_frequency);
	
	//
	//Calculate Counter value regarding to small Frequency
	//
		
	float period_time = 1.0 / 12000;
	float float_buzzer_timer_counter = (int)(1 / (desired_small_frequency * period_time));
	float float_buzzer_duty_cycle = float_buzzer_timer_counter / 2;
		
	buzzer_ctimer_counter = (int) float_buzzer_timer_counter - 1;
	buzzer_ctimer_duty_cycle = (int) float_buzzer_duty_cycle;
	
	//
	//Restart all Timers.
	//

  //
 	// Enable the timer interrupt in the NVIC.
  //
	// am_hal_interrupt_master_enable();
	
	//
  // Configure the pins for this example.
  //
	am_hal_gpio_pin_config(BUZZER_PIN, AM_HAL_PIN_12_TCTA0);
	am_hal_gpio_out_bit_set(BUZZER_PIN);
		
	//
	//Do the real Shit
	//
	init_cTimer();
	
	// am_util_stdio_printf("buzzer frequencies have been changed!");
}

//*****************************************************************************
//
// Initalize Buzzer CTimer function.
//
//*****************************************************************************
void
init_cTimer()
{
    //
    // Configure a timer to drive the BUZZER.
    //
    am_hal_ctimer_config_single(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA, (AM_HAL_CTIMER_FN_PWM_REPEAT | AM_HAL_CTIMER_HFRC_12KHZ |AM_HAL_CTIMER_INT_ENABLE | AM_HAL_CTIMER_PIN_ENABLE));

    //
    // Set up initial timer period.
    //
    am_hal_ctimer_period_set(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA, 64, 32);

    //
    // Enable interrupts for the Timer we are using on this board.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
    am_hal_interrupt_master_enable();
		
		
}

//*****************************************************************************
//
// Buzzer cTimer Interrupt Serive Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
  //
  // Clear the interrupt that got us here.
  //
  am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

  //
  // Now set new PWM half-period for the BUZZER.
  //
	
  am_hal_ctimer_period_set(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA, buzzer_ctimer_counter, buzzer_ctimer_duty_cycle);
	
	
}




//*****************************************************************************
//
// Buzzer Timer B1 Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr1_isr(void)
{
	
		am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE | AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
	
		BUZZER_WAKE_INTERVAL = BUZZER_XT_PERIOD * BUZZER_BUZZER_WAKE_INTERVAL_IN_MS * 1e-3;
    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
    am_hal_stimer_compare_delta_set(1, BUZZER_WAKE_INTERVAL);

		//
    // Toggle the cTimer.
    //
	
		toggle_bit = !toggle_bit;
		
		if(toggle_bit){
			am_hal_ctimer_start(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA);
		}	
		
		else{
			am_hal_ctimer_clear(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA);
		}
		
		am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
}
//*****************************************************************************
//
// Init function for Timer A0.
//
//*****************************************************************************
void
stimer_init_A(void)
{
    //
    // Enable compare A interrupt in STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA | AM_HAL_STIMER_INT_COMPAREB | AM_HAL_STIMER_INT_COMPAREC);

	
    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR0);
		am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR1);
		am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR2);
		
    //
    // Configure the STIMER and run
    //
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
		am_hal_stimer_compare_delta_set(1, BUZZER_WAKE_INTERVAL);
		am_hal_stimer_compare_delta_set(2, LCD_WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_A_ENABLE | 
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);

}
//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr0_isr(void)
{
		
		am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE | AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
		am_util_stdio_printf("%d" "\n", am_hal_stimer_counter_get());

		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(0));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(1));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(2));
    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
	
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(0));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(1));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(2));

		am_util_stdio_printf("%d" "\n", am_hal_stimer_counter_get());
	
		am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
    //
    // Read the pressure data
    //
		pressure_sensor_read();
	
		float x_old = xt[0];
	
		kalman_filter(data_pressure);
		
		// am_util_stdio_printf("\npressure: ");
		// am_util_stdio_printf("%f" " ", xt[0]);
		// am_util_stdio_printf("%f" " ", xt[1]);
		// am_util_stdio_printf("%d" " ", data_pressure);
	
		float velocity = calc_velocity(xt[0], x_old, data_temperature);
}


//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_compr2_isr(void)
{
	
		am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE | AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
		am_util_stdio_printf("%d" "\n", am_hal_stimer_counter_get());

		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(0));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(1));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(2));
		//
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
    am_hal_stimer_compare_delta_set(2, LCD_WAKE_INTERVAL);
	
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(0));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(1));
		am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(2));

		am_util_stdio_printf("%d" "\n", am_hal_stimer_counter_get());
	
		am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
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
    .ui8WriteThreshold = 12,
    .ui8ReadThreshold = 120,
};

//*****************************************************************************
//
// Configuration structure for the IO Master SPI.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig_spi =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_100KHZ,
    .bSPHA = 0,
    .bSPOL = 0,
};
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
		am_hal_iom_pwrctrl_enable(IOM_MODULE_SPI);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_config(IOM_MODULE_I2C, &g_sIOMI2cConfig_i2c);
		am_hal_iom_config(IOM_MODULE_SPI, &g_sIOMConfig_spi);

    //
    // Set pins high to prevent bus dips.
    //
    am_hal_gpio_out_bit_set(5);
    am_hal_gpio_out_bit_set(6);
	
	  am_hal_gpio_pin_config(8, AM_HAL_PIN_8_M1SCK);
    am_hal_gpio_pin_config(10, AM_HAL_PIN_10_M1MOSI);
    am_hal_gpio_pin_config(12, AM_HAL_PIN_12_M1nCE0);

#ifdef INTERNAL_LOOPBACK
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCLLB | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_SLSDALB | AM_HAL_GPIO_PULLUP);
    AM_REG(GPIO, LOOPBACK) = IOM_MODULE_I2C;
#else
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCL | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0SDA | AM_HAL_GPIO_PULLUP);
#endif

    am_hal_iom_int_enable(IOM_MODULE_I2C, 0xFF);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0);

    //
    // Turn on the IOM for this operation.
    //
    am_bsp_iom_enable(IOM_MODULE_I2C);
		am_bsp_iom_enable(IOM_MODULE_SPI);
}



//*****************************************************************************
//
// Initialize MS5611
//
//*****************************************************************************
void
pressure_sensor_init(void)
{
		am_util_stdio_printf("initializing sensor...\n");
		
		//
		// Send reset command
		//
		uint8_t cmd = CMD_RESET;
		uint32_t res = am_hal_iom_i2c_write(IOM_MODULE_I2C, MS5611_I2C_ADRESS, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		
		//
		// Check if reset was succesful
		//
		if (res != 0) {
				am_util_stdio_printf("initialization not succesfull\n");
		}
		else {
			
				am_util_stdio_printf("initialization succesfull\n");
			
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

		pressure_sensor_read();
		xt[0] = data_pressure;
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
		// Set up conversion mode for OSR 512 (pressure)
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
		
//		am_util_stdio_printf("\npressure: ");
//		am_util_stdio_printf("%" PRId64 "\n", P);
//		am_util_stdio_printf("\ntemperature: ");
//		am_util_stdio_printf("%d", TEMP);
		
		data_pressure = (int32_t)P;
		data_temperature = TEMP;
}
//*****************************************************************************
//
// Kalman Filter
//
//*****************************************************************************	
void
kalman_filter(int32_t data)
{
	
		// x = A*x
		xt[0] = xt[0] + dt*xt[1];
	
		// P = A*P*A'+Q
		P[0] += A[1]*(P[1]+P[2])+A[1]*A[1]*P[3]+Q[0];
		P[1] += A[1]*P[3] + Q[1];
		P[2] += A[1]*P[3] + Q[2];
		P[3] += Q[3];
	
		float y = data - xt[0];
		float S = P[0] + R;
		float K[] = {P[0]/S, P[2]/S};
		
		xt[0] = (xt[0] + (K[0]*y));
		xt[1] = (xt[1] + (K[1]*y));
		P[0] = (1-K[0])*P[0];
		P[1] = ((1-K[0])*P[1]);
		P[2] = (P[2]-K[1]*P[0]);
		P[3] = (P[3]-K[1]*P[1]);
}
//*****************************************************************************
//
// Initialize display
//
//*****************************************************************************
void
display_init(void) 
{
	
		//
    // Drive RESET low.
    //
    am_hal_gpio_out_bit_set(DRIVE_SLAVE_RESET_PIN);
    am_hal_gpio_pin_config(DRIVE_SLAVE_RESET_PIN, AM_HAL_PIN_OUTPUT);
		am_util_delay_ms(10);
		uint32_t cmd = 0x00;
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		am_hal_gpio_out_bit_clear(DRIVE_SLAVE_RESET_PIN);
		am_util_delay_us(2);
		am_hal_gpio_out_bit_set(DRIVE_SLAVE_RESET_PIN);
		
		am_hal_gpio_out_bit_clear(D_C_PIN);
		am_hal_gpio_pin_config(D_C_PIN, AM_HAL_PIN_OUTPUT);
		cmd = 0x21;							// Function Set H = 1
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 8, AM_HAL_IOM_RAW);
		cmd = 0xbf;							// Set Vop
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		cmd = 0x04;							// Set temperature coefficients
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		cmd = 0x13;							// Set LCD Bias mode
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		cmd = 0x20;							// Function Set H = 0, PD = 0, V = 0
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		cmd = 0x0c;							// Display control set normal mode D = 1, E = 0
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);

		
		am_hal_gpio_out_bit_set(D_C_PIN);
		for(int i = 0; i < 6*84; i++) {
			cmd = 0x00;							// DataWrite 
			am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		}
}

//*****************************************************************************
//
// Write String
//
//*****************************************************************************
void
LcdString(char *characters, uint8_t x, uint8_t y)
{
	
	am_hal_gpio_out_bit_clear(D_C_PIN);
	uint8_t cmd = 0x40 + y;
	am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
	cmd = 0x80 + y;
	am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
	am_hal_gpio_out_bit_set(D_C_PIN);
		
	//Note that char DataType is always terminated with a 0
  while (*characters)
  {
		for (int index = 0; index < 5; index++)
		{
	//uint32_t* pui32Data = ASCII[characters++ - 0x20][index]; ((*(matrix)) + (i * COLS + j))
			char character = ASCII[*characters - 0x20][index];
			am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&character, 1, AM_HAL_IOM_RAW);
		}
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&ASCII, 1, AM_HAL_IOM_RAW);
		characters++;
  }
}

//*****************************************************************************
//
// Calculate vertical velocity
//
//*****************************************************************************
float
calc_velocity(float x_new, float x_old, float temp)
{
		float sea_press = 101325;
		float altitude_new = 44330.0f * (1.0f - pow(x_new / sea_press, 0.1902949f));
		float altitude_old = 44330.0f * (1.0f - pow(x_old / sea_press, 0.1902949f));
	
		float diff_altitude = altitude_new - altitude_old;
		float vertical_speed = diff_altitude/WAKE_INTERVAL_IN_MS*1000;
	
		velocity_array[velocity_array_counter] = vertical_speed;
		velocity_array_counter = (velocity_array_counter + 1) % 10;
		float vertical_speed_avg = 0;
		for (int i = 0; i < 10; i++) vertical_speed_avg += velocity_array[i];
		vertical_speed_avg /= 10;
	
		char speed_string[10];
		
		am_util_stdio_sprintf((char *)&speed_string, "%.1f", vertical_speed_avg);
		am_util_stdio_printf("%s" "\n",(char *)&speed_string);

		LcdString((char *)&speed_string, 5, 3);
		// am_util_delay_ms(50);
 	
		return vertical_speed;
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
    // Enable Interrupts.
    //
    am_hal_interrupt_master_enable();

    //
    // Set up the IOM
    //
    iom_set_up();
		
		//
    // STIMER init.
    //
		
		// init_cTimer();
		// am_hal_gpio_pin_config(BUZZER_PIN, AM_HAL_PIN_12_TCTA0);
		// am_hal_gpio_out_bit_set(BUZZER_PIN);
    stimer_init_A();
		am_hal_interrupt_master_enable();
		
		
		
		//
		// Start the motherfucking Buzzy
		//
		

		// buzzer_change_frequency(1, 100);

		
		//
    // Initialize the sensor and read the coefficients
    //
		pressure_sensor_init();
		
		//
    // Initialize display
    //
		display_init();
		
    //
    // Loop forever.
    //
		
		
				
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

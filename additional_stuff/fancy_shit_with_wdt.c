#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "inttypes.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

#define BUZZER_PIN 12
#define BUZZER_PWM_TIMER 0

uint32_t WAKE_INTERVAL_IN_MS = 1000;
uint32_t XT_PERIOD = 32768;
uint32_t WAKE_INTERVAL = 32768 * 1000 * 1e-3;


//*****************************************************************************
//
// Buzz frequency, gives Dutycycle for Pulse
//
//*****************************************************************************
float buzzer_small_frequency = 1200;
uint32_t buzzer_ctimer_counter = 10, buzzer_ctimer_duty_cycle = 5;
uint32_t toggle_bit = 1;

//*****************************************************************************
//
// WatchDogStuff
//
//*****************************************************************************
uint8_t g_ui8NumWatchdogInterrupts = 0;
uint32_t g_ui32ResetStatus = 0;

am_hal_wdt_config_t g_sWatchdogConfig =
{

#ifdef AM_PART_APOLLO2
    .ui32Config = AM_REG_WDT_CFG_CLKSEL_1HZ | AM_HAL_WDT_ENABLE_RESET | AM_HAL_WDT_ENABLE_INTERRUPT,
#endif
    //
    // Set WDT interrupt timeout for 3 second.
    //
    .ui16InterruptCount = 3,

    //
    // Set WDT reset timeout for 15 seconds.
    //
    .ui16ResetCount = 15
};


//*****************************************************************************
//
// Function Declarations
//
//*****************************************************************************

void buzzer_shutdown(void);
void buzzer_start(void);
void buzzer_configure_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency);
void buzzer_change_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency);
void init_cTimer();
void am_ctimer_isr(void);
void pwm_stimer_init(void);
void am_stimer_cmpr1_isr(void);

void am_watchdog_isr(void);

//*****************************************************************************
//
// Buzzershutdown
//
//*****************************************************************************
void
buzzer_shutdown(void)
{
	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
	am_hal_ctimer_clear(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Buzzerstart
//
//*****************************************************************************
void
buzzer_start(void)
{
	//
	// STIMER init.
  //
  pwm_stimer_init();

  //
 	// Enable the timer interrupt in the NVIC.
  //
	am_hal_interrupt_master_enable();
	
	//
  // Configure the pins for this example.
  //
	am_hal_gpio_pin_config(BUZZER_PIN, AM_HAL_PIN_12_TCTA0);
	am_hal_gpio_out_bit_set(BUZZER_PIN);
		
	//
	//Do the real Shit
	//
	init_cTimer();
}


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
	
	WAKE_INTERVAL_IN_MS = 1000 * (1.0 / desired_big_frequency);
	
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
	// STIMER init.
  //
  pwm_stimer_init();

  //
 	// Enable the timer interrupt in the NVIC.
  //
	am_hal_interrupt_master_enable();
	
	//
  // Configure the pins for this example.
  //
	am_hal_gpio_pin_config(BUZZER_PIN, AM_HAL_PIN_12_TCTA0);
	am_hal_gpio_out_bit_set(BUZZER_PIN);
		
	//
	//Do the real Shit
	//
	init_cTimer();
	
	am_util_stdio_printf("buzzer frequencies have been changed!");
}

//*****************************************************************************
//
// Buzzer configure frequency
//
//*****************************************************************************
void
buzzer_configure_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency)
{
		//
		//Calculate Waketime for big Frequency
		//
	
		WAKE_INTERVAL_IN_MS = 1000 * (1.0 / desired_big_frequency);
	
		//
		//Calculate Counter value regarding to small Frequency
		//
		
		float period_time = 1.0 / 12000;
		float float_buzzer_timer_counter = (int)(1 / (desired_small_frequency * period_time));
		float float_buzzer_duty_cycle = float_buzzer_timer_counter / 2;
		
		buzzer_ctimer_counter = (int) float_buzzer_timer_counter - 1;
		buzzer_ctimer_duty_cycle = (int) float_buzzer_duty_cycle;
}
	

//*****************************************************************************
//
// Initalize CTimer function.
//
//*****************************************************************************
void
init_cTimer()
{
		//
    // Clear the terminal and print the banner.
    //
    am_util_stdio_printf("PWM ON BUZZER BITCH\n\n");
    am_util_delay_ms(10);

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
// Timer Interrupt Serive Routine (ISR)
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
// Init function for Timer B1.
//
//*****************************************************************************
void
pwm_stimer_init(void)
{
    //
    // Enable compare B interrupt in STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREB);

    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR1);

    //
    // Configure the STIMER and run
    //
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(1, WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE);

}

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr1_isr(void)
{
	
		WAKE_INTERVAL = XT_PERIOD * WAKE_INTERVAL_IN_MS * 1e-3;
    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
    am_hal_stimer_compare_delta_set(1, WAKE_INTERVAL);

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
}



//*****************************************************************************
//
// Interrupt handler for the WATCHDOG.
//
//*****************************************************************************
void
am_watchdog_isr(void)
{
    //
    // Clear the watchdog interrupt.
    //
    am_hal_wdt_int_clear();

    //
    // Catch the first four watchdog interrupts, but let the fifth through
    // untouched.
    //
    if ( g_ui8NumWatchdogInterrupts < 4 )
    {
        //
        // Restart the watchdog.
        //
        am_hal_wdt_restart();
    }

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Send a status message and give it some time to print.
    //
    am_util_stdio_printf("Interrupt #: %d\n", g_ui8NumWatchdogInterrupts);
    am_util_delay_ms(100);

    //
    // Increment the number of watchdog interrupts.
    //
    g_ui8NumWatchdogInterrupts++;


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
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("PWM once a second BITCH\n");
    am_util_delay_ms(10);

		//
		// Start the motherfucking Buzzy
		//
		
//		for(int i = 1; i < 1000; i++){
//			buzzer_change_frequency(i, 100 * i);
//			am_util_delay_ms(3000);
//		}
		
		
//*****************************************************************************
//
// Tryin some WatchdogStuff
//
//*****************************************************************************

		//
    // Clear reset status register for next time we reset.
    //
    am_hal_reset_status_clear();

    //
    // LFRC has to be turned on for this example because the watchdog only
    // runs off of the LFRC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Configure the watchdog.
    //
    am_hal_wdt_init(&g_sWatchdogConfig);

    //
    // Enable the interrupt for the watchdog in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_WATCHDOG);
    am_hal_interrupt_master_enable();

    //
    // Enable the watchdog.
    //
    am_hal_wdt_start();
		
//*****************************************************************************
//
// End of WatchdogStuff
//
//*****************************************************************************
		
		
    //
    // Sleep forever while waiting for an interrupt.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

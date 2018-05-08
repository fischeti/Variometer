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
uint32_t BUZZER_WAKE_INTERVAL_IN_MS = 1000;
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
int32_t data_temperature = 0;
uint32_t altitude = 0;
uint32_t coeff[] = {0,0,0,0,0,0,0,0};
int32_t pressure_and_temperature[] = {0,0};
float32_t pressure_old = 0;
float32_t altitude_old = 0;
float32_t velocity_array[] = {0,0,0,0,0,0,0,0,0,0};
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
,{0xF8, 0xF8, 0xF8, 0xF8, 0xF8} // 80 .
,{0x00, 0x07, 0x05, 0x07, 0x00} // 81 °
};


static const uint8_t ASCII_BIG[][15] = 
{
	 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 0 top
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 0 middle
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 0 bottom
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 1 top
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 1 middle
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 1 bottom
	,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 2 top
	,{0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F} // 2 middle
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8} // 2 bottom
	,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 3 top
	,{0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 3 middle
	,{0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 3 bottom
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 4 top
	,{0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 4 middle
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 4 bottom
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F} // 5 top
	,{0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE} // 5 middle
	,{0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 5 bottom
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F} // 6 top
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE} // 6 middle
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 6 bottom
	,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 7 top
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 7 middle
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 7 bottom
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 8 top
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 8 middle
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 8 bottom
	,{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 9 top
	,{0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 9 middle
	,{0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // 9 bottom
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // - top
	,{0x00, 0x00, 0x00, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x00, 0x00, 0x00} // - middle
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // - bottom
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // + top
	,{0x00, 0x00, 0x00, 0x38, 0x38, 0x38, 0xFF, 0xFF, 0xFF, 0x38, 0x38, 0x38, 0x00, 0x00, 0x00} // + middle
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // + bottom
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //   top
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //   middle
	,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //   bottom
	
};


//*****************************************************************************
//
// Kalman Initialization
//
//*****************************************************************************
float32_t P_init[] = {158806, 107, 124, 0.157};
const float32_t A_init[] = {1, 100, 0, 1};
const float32_t H_init[] = {1, 0};
const float32_t Q_init[] = {0.0025, 0.005, 0.005, 0.01};
float32_t K_init[] = {0, 0};
const float32_t R = 1e7;
float32_t xt_init[] = {1, 1};
const float32_t eye_init[] = {1, 0, 0, 1};

arm_matrix_instance_f32 P = {2, 2, (float32_t *)P_init};
arm_matrix_instance_f32 A = {2, 2, (float32_t *)A_init};
arm_matrix_instance_f32 H = {1, 2, (float32_t *)H_init};
arm_matrix_instance_f32 Q = {2, 2, (float32_t *)Q_init};
arm_matrix_instance_f32 K = {2, 1, (float32_t *)K_init};
arm_matrix_instance_f32 xt = {2, 1, (float32_t *)xt_init};
arm_matrix_instance_f32 eye = {2, 2, (float32_t *)eye_init};

//float dt = WAKE_INTERVAL_IN_MS;
//float xt[] = {0, 0, 0}; 

//float P[] = {158806, 107, 124, 0.157};
//float A[] = {1, 100, 0, 1};
//float H[] = {1, 0};
//float R = 1000000;
//float Q[] = {0.0025, 0.005, 0.005, 0.01};
//float eye[] = {1, 0, 0, 1};

//*****************************************************************************
//
// Function Declarations
//
//*****************************************************************************
void stimer_init(void);
void am_stimer_cmpr0_isr(void);
void init_watchdog(void);
void am_iomaster0_isr(void);
void itm_start(void);
static void iom_set_up(void);
void pressure_sensor_init(void);
void pressure_sensor_read(void);
void kalman_filter(uint32_t data);
void display_init(void);
void LcdString(char *characters, uint8_t x, uint8_t y);
void calc_velocity(float x_new, float x_old, float temp);
void buzzer_change_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency);
void init_cTimer(void);
void am_ctimer_isr(void);
void am_stimer_cmpr1_isr(void);
void write_big_number(uint32_t number, uint8_t x, uint8_t y);


am_hal_wdt_config_t g_sWatchdogConfig =
{
    //
    // Select the Apollo 2 Clock Rate.
    //

    .ui32Config = AM_REG_WDT_CFG_CLKSEL_128HZ | AM_HAL_WDT_DISABLE_RESET | AM_HAL_WDT_ENABLE_INTERRUPT,
	
    //
    // Set WDT interrupt timeout for 1 second.
    //
    .ui16InterruptCount = 128,
};

//*****************************************************************************
//
// BuzzerChange
//
//*****************************************************************************
void
buzzer_change_frequency(uint32_t desired_big_frequency, uint32_t desired_small_frequency)
{
	
	//
	//Enable Timer Pin
	//
	am_hal_ctimer_pin_enable(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA); 

	//
	//Calculate Waketime for big Frequency
	//
	BUZZER_WAKE_INTERVAL_IN_MS = 1000 * (1.0 / desired_big_frequency);
	
	
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
// Init function for Timer A0 & B1.
//
//*****************************************************************************
void
stimer_init(void)
{
    //
    // Enable compare A, B, (C) interrupt in STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA | AM_HAL_STIMER_INT_COMPAREB/* | AM_HAL_STIMER_INT_COMPAREC*/);

	
    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR0);
		am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR1);
		// am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR2);
		
    //
    // Configure the STIMER and run
    //
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
		am_hal_stimer_compare_delta_set(1, BUZZER_WAKE_INTERVAL);
		// am_hal_stimer_compare_delta_set(2, LCD_WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_A_ENABLE | 
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE/* |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE*/);

}
//*****************************************************************************
//
// Init Watchdog Timer
//
//*****************************************************************************
void
init_watchdog(void)
{
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
}
//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr0_isr(void)
{
		//
		// Freeze the sTimer counter
		//
		am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE | AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE/* |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE*/);

    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
	
		//
		// Increment compare Register A
		//
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);

	
		//
		// Continue sTimer counter
		//
		am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE/* |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE*/);
    
		//
    // Read the pressure data
    //
		pressure_sensor_read();
		
	
		pressure_old = *xt.pData;
		
		//
		// Apply a Kalman filter to the pressure data
		//
		kalman_filter(data_pressure);
		
		calc_velocity(*xt.pData, pressure_old, data_temperature);
}


//*****************************************************************************
//
// Buzzer Timer B1 Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr1_isr(void)
{
	
		//
		// Freeze the sTimer counter
		//
		am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE | AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE/* |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE*/);
	
	
		//
		// Set new Buzzer wake up interval
		//
		BUZZER_WAKE_INTERVAL = BUZZER_XT_PERIOD * BUZZER_WAKE_INTERVAL_IN_MS * 1e-3;
    
	
		//
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
	
	
		//
		// Increment compare Register B
		// 
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
		
		//
		// Continue sTimer counter
		//
		am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE/* |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE*/);
}
//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_compr2_isr(void)
{
		//
		// Freeze the sTimer counter
		//
		am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE | AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);


		//
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);
	
	
		//
		// Increment compare Register C
		// 
    am_hal_stimer_compare_delta_set(2, LCD_WAKE_INTERVAL);
	
		// am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(0));
		// am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(1));
		// am_util_stdio_printf("%d" "\n", am_hal_stimer_compare_get(2));
		// am_util_stdio_printf("%d" "\n", am_hal_stimer_counter_get());
	
	
		//
		// Continue sTimer counter
		//
		am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_B_ENABLE |
                         AM_HAL_STIMER_CFG_COMPARE_C_ENABLE);
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

		am_hal_wdt_restart();
	
	
		float32_t vertical_speed_avg = 0;
		for (int i = 0; i < 10; i++) vertical_speed_avg += velocity_array[i];
		vertical_speed_avg /= 10;

		// am_util_stdio_printf("%f\n", vertical_speed_avg);
		if ((int32_t)(vertical_speed_avg*10) != 0) {
			if (vertical_speed_avg < 0)
				write_big_number(10, 0, 0);
			else
				write_big_number(11, 0, 0);
		}
		else {
			write_big_number(12, 0, 0);
		}
		
		uint32_t first_digit = abs((int32_t)vertical_speed_avg) % 10;
		uint32_t second_digit = abs((int32_t)(vertical_speed_avg * 10)) % 10;

		write_big_number(first_digit, 16, 0);
		write_big_number(second_digit, 40, 0);
		
		char altitude_string[] = "     ";
		am_util_stdio_sprintf(altitude_string, "%d", altitude);
		altitude_string[3 + ((altitude / 1000) > 0)] = 'm';
		LcdString(altitude_string, 0, 4);
		
		char temperature_string[] = "     ";
		am_util_stdio_sprintf(temperature_string, "%d", data_temperature / 100);
		temperature_string[2 + (data_temperature < 0)] = 0x81;
		temperature_string[3 + (data_temperature < 0)] = 'C';
		LcdString(temperature_string, 40, 4);
		
		//
		//BuzzerFrequencyStuff
		//
		uint32_t abs_of_velocity = (uint32_t) fabs(vertical_speed_avg);
		
		uint32_t big_offset = 0;
		uint32_t big_gradient = 2;
		uint32_t big_frequency = big_gradient * abs_of_velocity + big_offset;
		
		// am_util_stdio_printf("This is the first digit: " "%d\n", first_digit);
		
		if(fabs(vertical_speed_avg) >= 0.5){
			// am_util_stdio_printf("Changing Frequency to: " "%d\n", big_frequency);
			
			buzzer_change_frequency(big_frequency, 200);
		}
		
		else {
			// am_util_stdio_printf(" No sound!\n");
			am_hal_ctimer_pin_disable(BUZZER_PWM_TIMER, AM_HAL_CTIMER_TIMERA);
		}


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
	
		//
		// Configure SPI pins
		//
	  am_hal_gpio_pin_config(8, AM_HAL_PIN_8_M1SCK);
    am_hal_gpio_pin_config(10, AM_HAL_PIN_10_M1MOSI);
    am_hal_gpio_pin_config(12, AM_HAL_PIN_12_M1nCE0);
	
		//
		// Configure I2C pins
		//
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCL | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0SDA | AM_HAL_GPIO_PULLUP);

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
		*xt.pData = data_pressure;
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
// Kalman Filter
//
//*****************************************************************************	
void
kalman_filter(uint32_t data)
{
			// Initialize temp matrices
			float32_t temp_2_1_init[] = {0, 0};
			float32_t temp_2_2_init[] = {0, 0, 0, 0};
			float32_t temp_2_2_t_init[] = {0, 0, 0, 0};
			
			// Declare temp matrices
			arm_matrix_instance_f32 temp_2_1 = {2, 1, (float32_t *)temp_2_1_init};
			arm_matrix_instance_f32 temp_2_2 = {2, 2, (float32_t *)temp_2_2_init};
			arm_matrix_instance_f32 temp_2_2_t = {2, 2, (float32_t *)temp_2_2_t_init};
			
			// x = A*x
			arm_mat_mult_f32(&A, &xt, &temp_2_1);
			for (int i = 0; i < 2; i++) *(xt.pData+i) = *(temp_2_1.pData+i);
//			am_util_stdio_printf("%.2f, %.2f, %.2f, %.2f\n", *A.pData, *(A.pData+1), *(A.pData+2), *(A.pData+3));
//			am_util_stdio_printf("%.2f, %.2f\n", *xt.pData, *(xt.pData+1));
//			am_util_stdio_printf("%.2f, %.2f\n", *temp_xt.pData, *(temp_xt.pData+1));
	
			
			// P = A*P*A'+Q
			arm_mat_trans_f32(&A, &temp_2_2_t);
			arm_mat_mult_f32(&A, &P, &temp_2_2);
			arm_mat_mult_f32(&temp_2_2, &temp_2_2_t, &P);
			for (int i = 0; i < 4; i++) *(temp_2_2.pData + i) = *(P.pData + i);
			arm_mat_add_f32(&temp_2_2, &Q, &P);
	
			// y = Z - (H*x)
			float32_t y = data - *xt.pData;
	
			// S=(H*P*H'+R)
			float32_t S = *P.pData + R;
	
			// K=P*H'*inv(S)
			arm_mat_trans_f32(&H, &K);
			arm_mat_mult_f32(&P, &K, &temp_2_1);
			arm_mat_scale_f32(&temp_2_1, 1/S, &K);
			
			// xt = xt + K*y
			arm_mat_scale_f32(&K, y, &temp_2_1);
			arm_mat_add_f32(&xt, &temp_2_1, &xt);
			
			// P = (I-K*H)*P
			arm_mat_mult_f32(&K, &H, &temp_2_2);
			arm_mat_sub_f32(&eye, &temp_2_2, &temp_2_2);
			arm_mat_mult_f32(&temp_2_2, &P, &temp_2_2_t);
			for(int i = 0; i < 4; i++) *(P.pData + i) = *(temp_2_2_t.pData + i);
			
			// am_util_stdio_printf("%f %d\n", *xt.pData, data);
//			
//			am_util_stdio_printf("%.2f\n", *xt.pData);
			
//		// x = A*x
//		xt[0] = xt[0] + dt*xt[1];
//	
//		// P = A*P*A'+Q
//		P[0] += A[1]*(P[1]+P[2])+A[1]*A[1]*P[3]+Q[0];
//		P[1] += A[1]*P[3] + Q[1];
//		P[2] += A[1]*P[3] + Q[2];
//		P[3] += Q[3];
//	
//		// y = Z - (H*x)
//		float y = data - xt[0];
//	
//		// S=(H*P*H'+R)
//		float S = P[0] + R;
//	
//		// K=P*H'*inv(S)
//		float K[] = {P[0]/S, P[2]/S};
//		
//		// xt = xt + K*y
//		xt[0] = (xt[0] + (K[0]*y));
//		xt[1] = (xt[1] + (K[1]*y));
//		
//		// P = (I-K)*P
//		P[0] = (1-K[0])*P[0];
//		P[1] = ((1-K[0])*P[1]);
//		P[2] = (P[2]-K[1]*P[0]);
//		P[3] = (P[3]-K[1]*P[1]);
//		
//		am_util_stdio_printf("%f %d\n", xt[0], data);
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
		
		//
		// Drive D_C Pin low to start command mode
		//
		am_hal_gpio_out_bit_clear(D_C_PIN);
		am_hal_gpio_pin_config(D_C_PIN, AM_HAL_PIN_OUTPUT);
	
		//
		// Configure display
		//
		cmd = 0x21;							// Function Set H = 1
		am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
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

		//
		// clear Display
		//
		am_hal_gpio_out_bit_set(D_C_PIN);
		for(int i = 0; i < 6*84; i++) {
			cmd = 0x00;							// DataWrite 
			am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
		}
		
		//
		// Initialize basic UI elements
		//
		char c = (char)0x80;
		LcdString(&c, 33, 2);
		LcdString("m/s", 57, 2);
}

//*****************************************************************************
//
// Write Big Number
//
//*****************************************************************************
void
write_big_number(uint32_t number, uint8_t x, uint8_t y)
{
		//
		// print out top, middle and bottom section of a number
		//
		for (int i = 0; i < 3; i++)
		{
				
				am_hal_gpio_out_bit_clear(D_C_PIN);		// command mode
				uint8_t cmd = 0x40 + y + i;						// set y adress
				am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
				cmd = 0x80 + x;												// set x adress
				am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
				am_hal_gpio_out_bit_set(D_C_PIN);			// data transfer mode 
			
				//
				// Send a horizontal section of a number
				//
				for (int j = 0; j < 15; j++) {
						am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&ASCII_BIG[3 * number + i][j], 1, AM_HAL_IOM_RAW);
				}
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
	
	am_hal_gpio_out_bit_clear(D_C_PIN);		// command mode
	uint8_t cmd = 0x40 + y;								// set y adress
	am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
	cmd = 0x80 + x;												// set x adress
	am_hal_iom_spi_write(IOM_MODULE_SPI, 0, (uint32_t *)&cmd, 1, AM_HAL_IOM_RAW);
	am_hal_gpio_out_bit_set(D_C_PIN);			// data transfer mode
	
	//
	// Send string to display
	// Note that char string is always terminated with a 0
	//
  while (*characters)
  {
		for (int index = 0; index < 5; index++)
		{
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
void
calc_velocity(float32_t x_new, float32_t x_old, float32_t temp)
{
		// 
		// Calculate altitude
		//
		float32_t sea_press = 101325;
		float32_t altitude_new = 44330.0 * (1.0 - pow(x_new / sea_press, 0.1902949));
		altitude = (uint32_t)altitude_new;
	
		//
		// Calculate vertical speed
		//
		float32_t diff_altitude = altitude_new - altitude_old;
		float32_t vertical_speed = diff_altitude/WAKE_INTERVAL_IN_MS*1000;
	
		//
		// Store altitude
		//
		altitude_old = altitude_new;
	
		//
		// Store last 10 veritcal speeds in array
		//
		velocity_array[velocity_array_counter] = vertical_speed;
		velocity_array_counter = (velocity_array_counter + 1) % 10;
	
		//
		// Store last 10 veritcal speeds in array and average it
		//
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
    // am_hal_interrupt_master_enable();

    //
    // Set up the IOM
    //
    iom_set_up();
		
		//
		// Configure the pins for this example.
		//
		am_hal_gpio_pin_config(BUZZER_PIN, AM_HAL_PIN_12_TCTA0);
		am_hal_gpio_out_bit_set(BUZZER_PIN);
		
		//
		// Init Timers
		//
    stimer_init();
		init_cTimer();
		init_watchdog();
		
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

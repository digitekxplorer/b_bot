// June 17, 2023
// B_Bot definitions

#ifndef DEFS_H
#define DEFS_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define TT_MOTORS

// Debug
//#define UART_LOG    // send debug message to UART
// Watchdog Log
//#define WDOG_LOG
// FreeRTOS task durations
//#define FTOS_LOG
// Vehicle Movement FSM Log
//#define FSM_LOG

// BLE buffer sizes
#define BLECMD_BUFFER_SIZE 64
#define CLIENT_MESSAGE_BUFFER_SIZE 128


// ***************************
// Global Vehicle structures
// ***************************
// structure: Global Vehicle parameters
// Initialized in main.c
// Used in: main.c, monitor.c, and veh_movmnt_fsm.c
// Using pointers to structures we have access to these parameters across multiple files.
typedef struct {
  uint32_t manual_cmd_mode;    // vehicle command mode:
                               // 0 = auto obstacle detection
                               // 1 = Vehicle right turn; Manual mode
                               // 2 = Vehicle left turn; Manual mode
                               // 3 = No auto obstacle detection; Manual mode;
  uint32_t dutyCycle_primary;  // PWM duty cycle for motor speed; forward and reverse
  uint32_t dutyCycle_turn;     // PWM duty cycle for motor speed; used when turning
  uint32_t veh_turn_dly;       // vechicle turn delay in 60 mSec increments
  bool     veh_fwd_active;     // vehicle in active auto forward mode (used in monitor.c, main.c, veh_movmnt_fsm.c, pico_init.c)
  bool     veh_turn_active;    // vehicle in active manual turn mode (used in monitor.c, main.c, veh_movmnt_fsm.c, pico_init.c)
  bool     led_pcb_on;         // B_Bot PCB LED status
}  Veh_params_t; 
extern Veh_params_t veh;      // Declaration of the global variable 'veh'
// Pointer to vehicle movement parameters structure for use with 'arrow' operator
#define veh_ptr ((Veh_params_t *)&veh)

// Example of structures used in timer.h
//#define timer_hw ((timer_hw_t *)TIMER_BASE)

/*
// Old method. It works but structure pointers is modeled after the structures in 
// the Raspberry Pi Pico SDK.
typedef struct {
  uint32_t manual_cmd_mode;    // vehicle command mode:
                               // 0 = auto obstacle detection
                               // 1 = Vehicle right turn; Manual mode
                               // 2 = Vehicle left turn; Manual mode
                               // 3 = No auto obstacle detection; Manual mode;
  uint32_t dutyCycle_primary;  // PWM duty cycle for motor speed; forward and reverse
  uint32_t dutyCycle_turn;     // PWM duty cycle for motor speed; used when turning
  uint32_t veh_turn_dly;       // vechicle turn delay in 60 mSec increments
  bool     active;             // vehicle in active command mode (used in monitor.c and main.c)
}  Veh_params_t;
//static Veh_params_t veh;              // structure name; for PC build
Veh_params_t veh;              // structure name
*/

// Motors Slice Numbers structure
// define structure to return multiple values
typedef struct {
    uint32_t slice_num1;
    uint32_t slice_num2;
} pwm_slice_t;
extern pwm_slice_t slnum;              // Declaration of the global variable 'slum'
// Pointer to motor slice number structure for use with 'arrow' operator
#define slnum_ptr ((pwm_slice_t *)&slnum)

//#define BLE_IN_SIZE 20
// BLE input, either commands or text to be display on SSD1306
// Use structure pointers to access shared BLE cmd and text structure members across multiple files
typedef struct {
  char ble_input[BLECMD_BUFFER_SIZE];        // commands from client (phone) placed here
  char client_message[CLIENT_MESSAGE_BUFFER_SIZE] ;  // text message displayed on SSD1306
  bool is_cltCmd                   ;  // do we have a client command
  bool is_cltTxt                   ;  // do we have a client message
  bool is_pcbLed                   ;  // did client press LED on or off
} Ble_cmd_text_t;
extern Ble_cmd_text_t blecmdtxt;        // Declaration of the global variable 'blecmdtxt'
// Pointer to the BLE cmd and text structure for use with 'arrow' operator.
#define blecmdtxt_ptr ((Ble_cmd_text_t *)&blecmdtxt)

// Example in .../pico-sdk/src/rp2040/hardware_structs/include/hardware/structs adc.h 
//#define adc_hw ((adc_hw_t *)ADC_BASE)

// ***************************
// Generic Definitions
// ***************************

// Pre-processor Directives Section
// FreeRTOS: The periods assigned to one-shot and auto-reload timers
#define mainONE_SHOT_TIMER_PERIOD       ( pdMS_TO_TICKS( 3333UL ) )
#define BLUE_LED_RELOAD_TIMER_PERIOD    ( pdMS_TO_TICKS( 250UL ) )
#define TaskDEFLTLED_DLY   250

// Watchdog timer reset
// Watchdog timeout duration in milliseconds (must be long enough for other tasks to execute)
#define WATCHDOG_TIMEOUT_MS           ( pdMS_TO_TICKS( 250UL ) )  // 250 mSec timeout
#define WATCHDOG_RELOAD_TIMER_PERIOD  ( pdMS_TO_TICKS( 100UL ) )  // 100 mSec timer update()

#define ADC_CHANNEL_TEMPSENSOR 4
#define FREERTOS_ENABLED true   // enable to use FreeRTOS delay function


// ***************************
// Vehicle Movements
// ***************************
// distance from HC-SR04 to an object in cm
#define MINDIS_STOPMTRS  15               // min distance is 15 cm, otherwise stop motors
//#define TURNDLY_90DEG    30

// ***************************
// Pico UART
// ***************************
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0    // GPIO0
#define UART_RX_PIN 1    // GPIO1

// ***************************
// Motors
// ***************************
// Default dutycycle to set speed (1 to 20)
// Duty cycle is related to speed, the higher the duty cycle the higher the speed,
// duty cycle proportional to speed.
#ifdef TT_MOTORS
// Dutycyles for new rover chassis with TT Motors
#define DCYCLE_PRIMARY  5
#define DCYCLE_TURN     5                   // set to turn speed for consistent speed during turns
// delay during turn, x/90
#define TURN_ANGLE      90                 // turn angle used in cmd_monitor.c
#define TURNDLY_90DEG   10.0               // this value compared to counter in veh_movmnt_fsm.c
#define TURN_COEFF      0.0778            // coeff to convert turn angle to 60mSec delays

// Original B_Bot motors
#else
#define DCYCLE_PRIMARY  3
#define DCYCLE_TURN     2                   // set to turn speed for consistent speed during turns
#define TURN_ANGLE      26                  // turn angle used in cmd_monitor.c
#define TURNDLY_90DEG   30.0
#define TURN_COEFF      .288                // coeff to convert turn angle to 60mSec delays
#endif

#define DCYCLE_MAX      20
// Motor nominal multiplers (nominal=250)
// Looking from behind the B_Bot and facing forward, mtr1 is on the right side, mtr2 is on left side
#define MTRDCYCLEMULTIPLIER1  250          // mtr1
#define MTRDCYCLEMULTIPLIER2  249          // mtr2

#define PERIOD   5000    // measured with scope: pwm freq=25KHz
#define DUTY_CYC_A DCYCLE_PRIMARY * MTRDCYCLEMULTIPLIER1      // mtr1
#define DUTY_CYC_B DCYCLE_PRIMARY * MTRDCYCLEMULTIPLIER2      // mtr2

// Motor1
#define MTR1_PWM_CHAN PWM_CHAN_A
#define MTR1_AIN1 8      // GPIO8
#define MTR1_AIN2 7      // GPIO7
#define MTR1_PWM  6      // GPIO6; PWM3A, Channel A
// Motor2
#define MTR2_PWM_CHAN PWM_CHAN_B
#define MTR2_BIN1 9      // GPIO9
#define MTR2_BIN2 10     // GPIO10
#define MTR2_PWM  11     // GPIO11; PWM5B, Channel B

// Default values to load rx_dat_ary (PIO ultrasonic readings) to prevent
// the vehicle from moving backward when the first forward command is sent
#define DEFLT_RXDATARY  61000


// ***************************
// HC-SR04 Ultrasonic Module
// ***************************
// output is 10 uSec trigger pulse on GPIO17
// input is ultrasonic echo return on GPIO16
#define HCSR04_ECHO  16  // GPIO16
#define HCSR04_TRIG  17  // GPIO17

#define SM_START_CMD        5

// HC-SR04 coefficient
#define HCSR04_COEFF       0.000136    // original


// ***************************
// LEDs and Switches
// ***************************
#define BLUE_LED  15       // GPIO15
#define BUTTON_GPIO 14    // GPIO14

// ***************************
// ADC
// ***************************
#define ADC_A0         26  // GPIO26; battery voltage divider
#define BATT_DIV_CHAN  0   // battery voltage divider connected to A0
#define LOW_BATT_WARNING_VAL  7.8   // low battery warning value
// ADC conversion factor
#define ADC_CONV_FACTOR   0.0000503548   // 3.3 / 65535
// Low voltage: voltage divider factor: (4.7K + 3K) / 3K = 2.5667
#define BATT_VOLT_DIV_FACTOR  2.5667 

// ADC number of conversion bits
#define ADC_NUM_BITS   12

// ***************************
// Bluetooth Low Engery (BLE)
// ***************************
// Buffer for Client commands; used in ftos_tasks.h and service_implementation.h
//#define BLE_IN_SIZE 100
//#define BLE_IN_SIZE 20

// External LEDs
 #define LED_EX1  18      // GPIO18

// ***************************
// SSD1306 Display
// ***************************
// Pico default pins are GPIO4 SDA0 and GPIO5 SCL0
// Used to communicate with SSD1306
#define I2C_PORT i2c_default
#define I2C_SDA_PIN 4    // GPIO4
#define I2C_SCL_PIN 5    // GPIO5

/* Example code to talk to a SSD1306 OLED display, 128 x 64 pixels
   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.
   Connections on Raspberry Pi Pico board, other boards may vary.
*/

// By default these devices are on bus address 0x3C or 0x3D. Check your documentation.
static int DEVICE_ADDRESS = 0x3C;

 // This can be overclocked, 2000 seems to work on the device being tested
 // Spec says 400 is the maximum. Try faster clocks until it stops working!
 // KHz.
#define I2C_CLOCK  1000

#define SSD1306_LCDWIDTH            128
#define SSD1306_LCDHEIGHT           64
#define SSD1306_FRAMEBUFFER_SIZE    (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8)

// Not currently used.
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10

#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22
#define SSD1306_DEACTIVATE_SCROLL   0x2E
#define SSD1306_ACTIVATE_SCROLL     0x2F

#define SSD1306_SETSTARTLINE        0x40

#define SSD1306_SETCONTRAST         0x81
#define SSD1306_CHARGEPUMP          0x8D

#define SSD1306_SEGREMAP0           0xA0
#define SSD1306_SEGREMAP127         0xA1
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF

#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB

#define TaskSSD1306_DLY             500

#endif   // DEFS_H

// June 17, 2023
// B_Bot definitions

#ifndef DEFS_H
#define DEFS_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

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
  bool     active;             // vehicle in active command mode (used in monitor.c and main.c)
}  Veh_params_t;
//static Veh_params_t veh;              // structure name; for PC build
Veh_params_t veh;              // structure name
// Pointer to vehicle movement parameters structure.
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

// structure: Vehicle Movement parameters
// Initialized in main.c
// Used in: main.c and veh_movmnt_fsm.c
// Using structures we have access to these parameters across multiple files.
typedef struct {
  uint32_t veh_state;          // vehicle movement state machine states
  float cm;                    // HC-SR04 distance measurement in centimeters
}  Fsm_params_t;               // Structure type
//static Fsm_params_t fsm;              // structure name; for PC build
Fsm_params_t fsm;              // structure name
// Pointer to vehicle movement parameters structure.
#define fsm_ptr ((Fsm_params_t *)&fsm)

/*
// Old method
typedef struct {
  uint32_t veh_state;          // vehicle movement state machine states
  float cm;                    // HC-SR04 distance measurement in centimeters
}  Fsm_params_t;               // Structure type
//static Fsm_params_t fsm;              // structure name; for PC build
Fsm_params_t fsm;              // structure name
*/

// Motors Slice Numbers structure
// define structure to return multiple values
typedef struct {
    uint32_t slice_num1;
    uint32_t slice_num2;
} pwm_slice_t;
pwm_slice_t slnum;
// Pointer to motor slice number structure.
#define slnum_ptr ((pwm_slice_t *)&slnum)
/*
// Old method to define motor slice number structure
//static struct pwm_slice_t {         // structure for PC build
struct pwm_slice_t {
    uint32_t slice_num1;
    uint32_t slice_num2;
//} slnum;                 // struct name
} ;
extern struct pwm_slice_t slnum;
*/

#define BLE_IN_SIZE 20
// BLE input, either commands or text to be display on SSD1306
// Use structure pointers to access shared BLE cmd and text structure members across multiple files
typedef struct {
  char ble_input[BLE_IN_SIZE];        // commands from client (phone) placed here
  char client_message[BLE_IN_SIZE] ;   // text message displayed on SSD1306
} Ble_cmd_text_t;
Ble_cmd_text_t blecmdtxt;
// Pointer to the BLE cmd and text structure.
#define blecmdtxt_ptr ((Ble_cmd_text_t *)&blecmdtxt)
// Example in .../pico-sdk/src/rp2040/hardware_structs/include/hardware/structs adc.h 
//#define adc_hw ((adc_hw_t *)ADC_BASE)

/*
// A different way to access a shared structure across multiple files using extern
struct Ble_cmd_text_t {
  char ble_input[BLE_IN_SIZE];        // commands from client (phone) placed here
  char client_message[BLE_IN_SIZE] ;   // text message displayed on SSD1306
} ; 
extern struct Ble_cmd_text_t blecmdtxt;   // declare BLE input structure as extern
*/

// ***************************
// Generic Definitions
// ***************************
// Period with which we'll enter the BTstack timer callback
#define HEARTBEAT_PERIOD_MS 250

// Pre-processor Directives Section
// FreeRTOS: The periods assigned to one-shot and auto-reload timers
#define mainONE_SHOT_TIMER_PERIOD       ( pdMS_TO_TICKS( 3333UL ) )
#define BLUE_LED_RELOAD_TIMER_PERIOD  ( pdMS_TO_TICKS( 250UL ) )
#define TaskDEFLTLED_DLY   250

#define ADC_CHANNEL_TEMPSENSOR 4
#define FREERTOS_ENABLED true   // enable to use FreeRTOS delay function

// Debug
#define PRINT_MESS    // // print debugging message

// ***************************
// Vehicle Movements
// ***************************
// distance from HC-SR04 to an object in cm
#define MINDIS_STOPMTRS  10               // min distance is 10 cm, otherwise stop motors
#define TURNDLY_90DEG    30
// Pre-processor Directives Section
// Auto-detectition and Control Vechicle states
#define VEHSTOP_GOFOR_AUTO      1        // Vehicle stop, then go forward
#define VEHFORWARD_AUTO         2        // Vehicle direction set to forward
#define VEHSTOP_INITBWD_AUTO    3        // Initiate backward movement
#define VEHSTOP_GOBACK_AUTO     4        // Vehicle stop, then go backward
#define VEHBACKWARD_AUTO        5        // Vehicle direction set to  backward
#define VEHSTOP_INITRIGHT_AUTO  6        // Vehicle stop, turn right
#define VEHSTOP_TURNRIGHT_AUTO  7        // Vehicle stop, turn right

// Manual Command, Right Turn FSM
#define VEHFORWARD_MAN1         8
#define VEHSTOP_INITRIGHT_MAN1  9
#define VEHSTOP_TURNRIGHT_MAN1  10
// Manual Command, Left Turn FSM
#define VEHFORWARD_MAN2         11
#define VEHSTOP_INITLEFT_MAN2   12
#define VEHSTOP_TURNLEFT_MAN2   13
// Manual Command, Backward Movement FSM
#define VEHFORWARD_MAN3         14
#define VEHSTOP_GOBACK_MAN3     15
#define VEHBACKWARD_MAN3        16

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
#define DCYCLE_PRIMARY  3
#define DCYCLE_MAX      20
#define DCYCLE_TURN     2                   // set to turn speed for consistent speed during turns
// Motor nominal multiplers (nominal=250)
#define MTRDCYCLEMULTIPLIER1  250          // mtr1
#define MTRDCYCLEMULTIPLIER2  250          // mtr2

#define PERIOD   5000    // measured with scope: pwm freq=25KHz
#define DUTY_CYC_A DCYCLE_PRIMARY * MTRDCYCLEMULTIPLIER1      // mtr1
#define DUTY_CYC_B DCYCLE_PRIMARY * MTRDCYCLEMULTIPLIER2      // mtr2

// Motor1
#define MTR1_PWM_CHAN PWM_CHAN_A
#define MTR1_AIN1 8      // GPIO8
#define MTR1_AIN2 7      // GPIO7
#define MTR1_PWM  6      // GPIO6; PWM3A, Channel A
// Motor2
//#define MTR2_BIN1 12     // GPIO12
//#define MTR2_BIN2 11     // GPIO11
//#define MTR2_PWM  10     // GPIO10; PWM5A

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
#define LOW_BATT_WARNING_VAL  6.0   // low battery warning value
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

//static char ble_input[BLE_IN_SIZE] ;      // commands from client (phone) placed here
// used in service_implementation.h and ftos_tasks.h
//static char client_message[BLE_IN_SIZE] = "Hi Alfredo" ; // initial message displayed on SSD1306

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

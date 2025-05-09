// July 25, 2024
// Author: Al Baeza
// B_Bot project using FreeRTOS and BTstack
// This project code is reuse code from A_bot however it has been updated for
// the Raspberry Pi Pico and FreeRTOS. The following tasks are performed by the 
// B_Bot: drive motors, blink LED(s), measure ultrasonic signals, provide user 
// interface during development, and provide wireless communications (either 
// Bluetooth or WiFi).
// 
// An Android App is needed to communicate with Pico's BTstack (Bluetooth).
// The Android App LightBlue by PunchThrough was used in this project.
//
// - UART inteface: incorporate UART RX interrupt with deferred task to read 
// input from the user.
// - Ultrasonic ranging: use Raspberry Pi Pico's unique feature called Programmable
// Input/Output (PIO) to drive HC-SR04 trigger signal and measure echo returns.
// 
// Useful commands for RP2040:
// openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program b_bot.elf reset exit"
//
// Useful commands for RP2350 using Raspberry Pi Debug Probe:
// sudo openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" 
// -c "rp2350.dap.core1 cortex_m reset_config sysresetreq" -c "program ftos_ble.elf verify reset exit"
// or in one line
// sudo openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "rp2350.dap.core1 cortex_m reset_config sysresetreq" -c "program b_bot.elf verify reset exit"
// Minicom terminal command
// minicom -b 115200 -o -D /dev/ttyACM0
// or
// Use following with USB to UART converter
// minicom -b 115200 -o -D /dev/ttyUSB0
//
// Useful links:
// https://www.raspberrypi.com
// https://github.com/raspberrypi
// https://freertos.org/Documentation/RTOS_book.html
// https://github.com/raspberrypi/FreeRTOS-Kernel/tree/main
// https://bluekitchen-gmbh.com/btstack
// https://punchthrough.com/lightblue/

// Releases:
// 10/28/2024  A. Baeza  Initial release: V1.0
// 01/23/2025  A. Baeza  See notes below: V1.1
// 02/14/2025  A. Baeza  Major release for Pico2 W (RP2350) build

// Updates:
// July 26, 2024
// A base sofware project has been created for the B_Bot rover.  The base design
// includes FreeRTOS for the Raspberry Pi Pico and an RX UART interrupt with a
// synchronized deferred Task to minimize time in the Interrupt Service Routine (ISR).
// July 29, 2024
// RX UART FIFO interrupt working as I expect.  Had to modify the function 
// uart_set_irq_enables(UART_ID, false, false). New function uart_set_irq_en_wo_timout()
// does not enable RX UART timeout interrupt to allow the user time to input a command.
// July 30, 2024
// Added motor.c and motor.h to the project.  Functions to initialize the motor 
// GPIOs for PWM are in the files. Also included are functions to control TB6612FNG
// Motor Driver with three Pico GPIO pins (one for PWM output and two to control the
// TB6612FNG).
// Aug 6, 2024
// Added HC-SR04 Ultrasonic module to project for collision avoidance.
// The HC-SR04 works by sending it a 10 us pulse on its Trigger pin.
// The distance to an object is represented by the length of the 
// pulse on its Echo pin. Programmable Input Output (PIO) used to 
// control the HC-SR04 pins.
// Aug 9, 2024
// Added printing functions for integer and floating point numbers using uart_putc().
// Aug 13, 2024
// Added Vehicle Movement Finite State Machines to control the movement of the 
// motors.
// Aug 14, 2024
// Added a structure in defs.h to define vehicle parameters.
// Aug 15, 2024
// Moved Vehicle Movement FSM to a separate file called veh_movmnt_fsm.c.
// Aug 16, 2024
// Cleaned up files. Added controls for second motor.
// Aug 18, 2024
// Small updates in motor.c to turn off second motor. First time test with both HC-SR04
// and motors.  Motors stop and reverse when ultrasonic measurements are less than 10 cm.
// Aug 26, 2024
// Switched to Pico W build process. The following changes are required to start using the
// Pico W in preparations for Wireless communications.
// Code and Cmake differences between Pico and Pico W
// Parent CMakeList.txt: set(PICO_BOARD pico_w)
// Project CMakeList.txt: target_link_libraries(${PROJECT_NAME} pico_stdlib pico_cyw43_arch_none)
// In main.c: #include "pico/cyw43_arch.h" and cyw43_arch_init() in pico_init.c
// Default LED: cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
// In pico_init.c and pico_init.h: int pico_led_init(void) for difference in default LED
// Sept 2, 2024
// Changed HC-SR04 Ultrasonic Module pin definitions in defs.h
// output is 10 uSec trigger pulse on GPIO17
// input is ultrasonic echo return on GPIO16
// Sept 3, 2024
// Changed external LED pin and pushbutton pin.  Added push-button interrupt.
// Pushbutton -> GPIO2 (used to move vehicle forward; manual forward command)
// Blue LED   -> GPIO3
// Sept 5, 2024
// Added halt ('hlt') to stop motors and ultrasonic ranging. Adjust motor dutycycle for speed.
// Sept 8, 2024
// Added separate speed for turns for a consistent speed during turns.
// Sept 29, 2024
// Added Bluetooth Low Energy (BLE) capabilities, using BTstack, to B_Bot main branch
// Oct 1, 2024
// Added vehicle commands
// Oct 3, 2024
// Reordered commands; code cleanup
// Oct 6, 2024
// Added SSD1306 Display functions
// Oct 7, 2024
// Updated ssd1306_display.c and ssd1306_display.h with code for previous 'combo' project found
// in FreeRTOS folder. 
// Oct 10, 2024
// Added Pico temperature characteristic to BLE
// Oct 14, 2024
// Debugging motor PWM in Oct 10th build. veh.dutyCycle_primary in veh structure was set to zero so
// the motor PWM stopped sending pulses. Removed static from structures in defs.h
// Oct 15, 2024
// Modified en_pwm_cmd() and dis_pwm_cmd() in monitor.c
// Oct 16, 2024
// Code cleanup: monitor.c
// Possible problem with UART delay at start of main.c code.
// Oct 17, 2024
// Added characteristic_e to send BLE text from phone to B_Bot.
// Oct 18, 2024
// Both commands and text messages, displayed on SSD1306, from client (phone) working.
// Oct 19, 2024
// FreeRTOS reload timer used for heartbeat counter and to update SSD1306 display
// Oct 21, 2024
// Use structure pointers to access shared BLE cmd and text structure members across multiple files
// Oct 22, 2024
// Updated all structures in defs.h with structure pointers
// Oct 23, 2024
// Code cleanup; removed vTaskSSD1306()
// Oct 26, 2024
// Moved some source files to src/rpp_bot subdirectory
// Oct 27, 2024
// Moved vBTstack_HandlerTask() to main.c and added get_xBTstack_HandlerTask() function.
// Major Version Release V1.0 GitHub.com/digitekxplorer
// Jan 5, 2025
// RP2350- E9 correction: use pull-up because of error in RP2350; see pico_init.c
// Changed BUTTON_GPIO to GPIO14 in defs.h
// Changed pin definitions: MTR2_BIN1=GPIO9; MTR2_BIN2=GPIO10; MTR2_PWM=GPIO11
// Jan 8, 2025
// Modified motor.c to include required change of GPIO11 to PWM_CHAN_B.
// Minor Version Release V1.1 GitHub.com/digitekxplorer
// Jan 23, 2025
// Added ADC connection to monitor and display battery voltage
// Jan 25, 2025
// Changed motor dutycyles for new rover chassis
// Jan 29, 2025
// Modified monitor commands to adjust vehicle speed and turns
//
// **********************************************************
// Release V2.0.0
// Feb 14, 2025
// Build for Pico2 W with RP2350
// Modified defs.h: added static to structure definitions
// Modified main.c: removed used header files like m0plus.h, not needed for RP2350
//
// Changes for this release to upgrade B_Bot to Pico2 W (RP2350)
// Upgraded to Raspberry Pi Pico SDK 2.1.0
// Upgraded FreeRTOS to use RP2350
// https://github.com/raspberrypi/FreeRTOS-Kernel/tree/main
// Verify we are using correct RP2350 port by checking code at the following link:
// https://github.com/raspberrypi/FreeRTOS-Kernel/tree/main/portable/ThirdParty/GCC/RP2350_ARM_NTZ

// Four other files modified for Pico2 W (RP2350): 
// Project level files: CMakeLists.txt, FreeRTOS_Kernel_import.cmake, and pico_sdk_import.cmake
// Source level files: CmakeLists.txt
// port level: FreeRTOSConfig.h
//
// Feb 18, 2025
// With suggestions from Google AI Studio, I added "extern" to structures in defs.h and
// defined the structures as global variables in main.c
// Feb 19, 2025
// Modified motor parameters in defs.h for turns and straight motion:TURNDLY_90DEG 
// and MTRDCYCLEMULTIPLIER1 (or MTRDCYCLEMULTIPLIER2)
// Feb 20, 2025
// Added font characters for SSD1306 display
// Feb 22, 2025
// Added more SSD2306 symbols to ssd1306_font.h and ssd1306_display.c
// Useful website for symbol creation: 
// https://stmn.itch.io/font2bitmap
// Mar 12, 2025
// Working on BLE functions. Modified server_gattfile.gatt, service_implementation.h
// Mar 16, 2025
// Updated vBTstack_HandlerTask(). 3 options: command, message, Led control
// Mar 17, 2025
// Updated service_implementation.h per suggestions from Google Gemini to define 
// string buffers and buffer sizes. Limit buffer copy to max buffer sizes.
// Mar 24, 2025
// Added Watchdog reset; timer task in FreeRTOS; define timeout in defs.h
// April 9, 2025
// After extensive testing trying to fix compatibility issue between FreeRTOS
// and BTstack, reverted back to code from March 24, 2025
// April 12, 2025
// Moved all BTstack setup and functions to vBTstack_HandlerTask() and removed
// bleServerTask(). Also removed bstack_run_loop_execute() call becuase the
// RP2350's PIO handles Transport Layer functions to move data between the CYW43439
// and the RP2350. No BTstack run loop is needed for the RP2350.
// April 15, 2025
// Change task name from vBLEinput_HandlerTask() to vBTstack_HandlerTask(). Added
// 3D time matrix to record FreeRTOS task times.
// April 16, 2025
// Added task timer to BTstack FreeRTOS task.  Found UART debug message in 'FWD' 
// command that was causing huge delays (in cmd_monitor.c). Remove UART debug message.
// Fixed Left turn issue in motor.c.
// April 21, 2025
// Code cleanup
// April 29, 2025
// cmd_monitor.c: fixed string to integer function in veh_speed_cmd() and veh_turn_dly_cmd()
// May 1,2025
// hcsr04.pio (HC-SR04 sensor): updated to use higher clock rate of 150MHz for RP2350
// May 5, 2025
// Major update to veh_movmnt_fsm.c. Replaced multiple FSMs with one state machine to 
// control movement.
// May 8, 2025
// Update to veh_movmnt_fsm.c, input to state IDLE: veh_ptr->manual_cmd_mode
// May 9, 2025
// Fixed manual reverse and code cleanup


/*
Copyright (c) <2025> <Al Baeza>

Permission is hereby granted, free of charge, to any person obtaining a copy 
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is furnished 
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
//#include "hardware/regs/m0plus.h"    // ab rp2040
//#include "hardware/regs/m33.h"       // ab rp2350
//#include "hardware/address_mapped.h" // ab not used
#include "hardware/adc.h"
// SSD1306 Display
#include "hardware/i2c.h"
#include "rpp_bot/ssd1306_font.h"
#include "rpp_bot/ssd1306_display.h"
// PIO assembly header for HC-SR04 UltraSound
#include "hcsr04.pio.h"
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
// BTstack
#include "btstack.h"
// High-level libraries
#include "pico/btstack_cyw43.h"
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
// ab
#include "pico_init.h"
#include "cmd_monitor.h"
#include "motor.h"
#include "rpp_bot/print_num.h"
#include "veh_movmnt_fsm.h"
#include "defs.h"
// GAP and GATT
#include "GATT_GAP/gap_config.h"               // GAP advertisement
#include "GATT_GAP/service_implementation.h"
#include "ftos_ble_task.h"

// Definition of the global variable in defs.h
Veh_params_t veh;           // global variable 'veh'
pwm_slice_t slnum;          // global variable 'slum'
Ble_cmd_text_t blecmdtxt;   // global variable 'blecmdtxt'

// *************
// FreeRTOS 
// *************
// There are six FreeRTOS tasks where one is a timer task.  The timer task has
// three timers that will trigger the FreeRTOS timer task.
// The FreeRTOS tasks are:
// 1) vBTstack_HandlerTask             - Bluetooth
// 2) vTaskBlinkDefault                - Default blinking LED (Pico on-board LED)
// 3) vUartRxDeferredIntrHandlerTask   - UART used for logging during debug
// 4) vPioRxDeferredIntrHandlerTask    - PIO used for HC-SR04 ultrasonic ranging
// 5) vPshbttnDeferredIntrHandlerTask  - pushbutton interrupt
// 6) prvTimerCallback                 - FreeRTOS timers
//    a) xTimer1Started                - one-shot timer for testing
//    b) xTimerBlueLedStarted          - second blinking LED
//    c) xTimerWatchdogStarted         - Watchdog timer
//
// The timer handles are used inside the callback function so 
// these timers are given file scope.
static TimerHandle_t xOneShotTimer;           // one shot (for testing)
static TimerHandle_t xBlueLedReloadTimer;     // LED
static TimerHandle_t xWatchdogReloadTimer;    // watchdog

// Function Prototypes for FreeRTOS Tasks and Callbacks
// User input using BLE either commands or text messages; BTstack Handler Task
void vBTstack_HandlerTask( void * pvParameters );
// The blinking LED function; Pico default LED
static void vTaskBlinkDefault(); 
// RX UART Deferred Task
static void vUartRxDeferredIntrHandlerTask( void * pvParameters );
// PIO RX FIFO Deferred Interrupt Handler Task
static void vPioRxDeferredIntrHandlerTask( void * pvParameters );
// Pushbutton Deferred Interrupt Handler Task
static void vPshbttnDeferredIntrHandlerTask( void * pvParameters );

// The callback function that is used by both timers.
static void prvTimerCallback( TimerHandle_t xTimer );

// UART RX interrupt handler
static void vUartRxInterruptHandler();
// PIO RX FIFO interrupt handler
static void vPioRxInterruptHandler();
// Pushbutton interrupt handler
static void vPshbttnInterruptHandler();

// Stores the handle of the task to which interrupt processing is deferred.
static TaskHandle_t xUartRXHandlerTask = NULL;
static TaskHandle_t xPioRXHandlerTask = NULL;
static TaskHandle_t xPshbttnHandlerTask = NULL;
static TaskHandle_t xBTstack_HandlerTask = NULL;   // used in service_implementation.h


// *****
// BLE
// *****
// BTstack objects
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Some data that we will communicate over Bluetooth
static int hb_counter = 0 ;

// ********************
// Function prototypes and local variables
// ********************
// UART RX
static void uart_irq_setup(int UART_IRQ);
static inline void uart_set_irq_en_wo_timout(uart_inst_t *uart, bool rx_has_data);
// PIO RX FIFO.
static void pio_irq_setup(void);

// PIO global variables
static volatile uint32_t pio_rx_irq_counter = 0;
static volatile uint32_t rx_dat_ary[4];
static volatile uint32_t rx_dat_avg = 0;

// ADC Pico poll for temperature and battery voltage
void adc_poll();            // temperature reading
// Get ADC measurement
float get_adc_reading(uint32_t chan); 
float deg_f;                 // temperature in deg F
float deg_c;                 // temperature in deg C
float temp_reading;          // temperature reading
float batt_volt_flt;         // battery voltage

// pushbutton
bool pshbttn_fwd = false;    // pushbutton indication

//----------
// Debug
//----------
void dbg_print(uint8_t chr);  // print a single character
// print text and float number
void dbg_uart_print(float uart_float_num, const char *uart_text) ; // print using UART
const char volt_message[] = "Battery voltage = ";      // text for battery voltage
const char temp_message[] = "Temperature deg F = ";    // text for temperature

int callbck_cnt = 0;

// Get the time since boot in microseconds using the Pico SDK function
uint64_t time_us = 0;
uint64_t diff_us = 0;
static char us_str[20];

// --- Time Matrix Declaration ---
// timers to calculate time duration for FreeRTOS tasks or interrupts.
// Time duration example: diff_us = time_matx[rr][0][1] - time_matx[rr][0][0] for first timer
#define ROWS 2    // start time [0] and end time [1]; subtract start time from end time
#define COLS 5    // number of timer durations for different tasks
#define TIM  8    // how many times to sample time to get an average if needed
// Declare a 8x2x5 time matrix of int64_t to calculate time delays(64-bit unsigned integer)
uint64_t time_matx[TIM][ROWS][COLS];

// *****************************************************************
// *****************************************************************
// Main
// *****************************************************************
int main() {
    stdio_init_all();   // instead of using this function initialize UART manually

    // Watchdog reset
    if (watchdog_enable_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
//        return 0;
    } else {
        printf("Clean boot\n");
    }

    // ****************
    // Pico Setup
    // ****************
    pico_io();          // setup Pico external LEDs, mtr IO, ...
    pico_uart_init();   // setup UART
    print_welcome(1);   // send welcome message to UART; number of seconds to delay
    
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mpu6050_i2c example requires a board with I2C pins
  puts("Default I2C pins were not defined");
  dbg_print('A');
#else
  puts("Default I2C pins ARE defined");
  dbg_print('B');
#endif

    // Initialize cyw43 and print message if failed.
    if (cyw43_arch_init()) {
        uart_puts(UART_ID, "BLE init failed.");
        return -1;
    }

    // Get the time since boot in microseconds using the Pico SDK function
    // Subtract start time from end time to get a duration and use the loop
    // to calculate an average.
#ifdef UART_LOG         
    for (int rr; rr<=4; rr++){
      time_matx[rr][0][0] = time_us_64();
      uart_puts(UART_ID, "Time since boot in uSec = ");
      sprintf(us_str, "%d", time_matx[rr][0][0]) ;      // convert to a string 
      uart_puts(UART_ID, us_str);
      uart_puts(UART_ID, "\r\n");

      time_matx[rr][0][1] = time_us_64();
      sprintf(us_str, "%d", time_matx[rr][0][1]) ;      // convert to a string 
      uart_puts(UART_ID, us_str);
      uart_puts(UART_ID, "\r\n");
      uart_puts(UART_ID, "Difference in uSec = ");
      diff_us = time_matx[rr][0][1] - time_matx[rr][0][0];
      sprintf(us_str, "%d", diff_us) ;      // convert to a string 
      uart_puts(UART_ID, us_str);
      uart_puts(UART_ID, "\r\n");
    }
#endif


    // Clear TB6612FNG motor driver AIN1&2 and BIN1&2 to have motors start in off state
    motors_off();

    // ****************
    // System Structures Setup
    // ****************
    // Initialize Vehicle parameters; includes command mode and motor parameters
    veh_stuct_init();

    // ****************
    // Command monitor
    // ****************    
    monitor_init();
    app_set();                  // initialize command array
    
    // ****************
    // Motors setup
    // ****************
    motor_init();    // start pwm running at pins
  
    // ****************
    // HC-SR04 Ultrasonic Module setup
    // ****************
    // UART RX configuration
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    // setup UART RX interrupt and enable
    uart_irq_setup(UART_IRQ);
    
    // Programmable Input Output (PIO) configuration
    // Choose which PIO instance to use (there are two instances)
    PIO pio = pio0;
    // Load PIO assembled program 
    uint offset = pio_add_program(pio, &hcsr04_program);
    // Find a free state machine
    uint sm = pio_claim_unused_sm(pio, true);
    // initialize the HC-SR04 PIO assembly(Echo pin = 14, Trig pin = 15)
    hcsr04_program_init(pio, sm, offset, HCSR04_ECHO, HCSR04_TRIG);
    // setup PIO RX FIFO interrupt and enable
    pio_irq_setup();
    
    // ****************
    // SSD1306
    // ****************
    i2c_init(I2C_PORT, I2C_CLOCK * 1000);
    
    SSD1306_initialize();         // initialize SSD1306
    ClearDisplay();               // clear SSD1306 display
    sleep_ms(500);                // short delay
    ssd1306_drawborder();
    WriteString(13,8,"HELLO B_Bot"); 
    UpdateDisplay();
    
    // ****************
    // Initialise adc for the temp sensor
    // ****************
    adc_init();
    static char adc_str[10];
    // temperature channel
    adc_set_temp_sensor_enabled(true);
    temp_reading = get_adc_reading(ADC_CHANNEL_TEMPSENSOR);     // temperature
    
    deg_c = 27 - (temp_reading - 0.706) / 0.001721;
    deg_f = (deg_c * 1.8) + 32.0;
    dbg_uart_print(deg_f, temp_message) ;     // print using UART
    
    // Battery voltage measurement
    batt_volt_flt = get_adc_reading(0);     // ADC A0 for battery voltage
    dbg_uart_print(batt_volt_flt, volt_message) ;   // print using UART

    
    // ****************
    // Pushbutton setup
    // ****************
    // Pushbutton connected to GPIO2
    // RP2350- E9 correction: use pull-up because of error in RP2350
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &vPshbttnInterruptHandler);

    // ****************
    // Watchdog Enable
    // ****************
    // Configure and start the watchdog timer
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1); // Timeout in milliseconds, pause on debug = true

    // *************************************************
    // FreeRTOS setup
    // *************************************************
    // Timer tasks
    BaseType_t xTimer1Started;
    // Blinking LED timer
    BaseType_t xTimerBlueLedStarted;
    // Watchdog timer update()
    BaseType_t xTimerWatchdogStarted;
    

    // ****************
    // FreeRTOS Tasks
    // ****************
    // One-Shot timmer: Create the one shot timer, storing the handle to the created timer in xOneShotTimer.
    // TODO: Currently this timer is not used for anything, so it can be removed in the future.
    xOneShotTimer = xTimerCreate( "OneShot",             // Text name for the timer - not used by FreeRTOS
                              mainONE_SHOT_TIMER_PERIOD, // The timer's period in ticks
                              pdFALSE,                   // Set uxAutoRealod to pdFALSE to create a one-shot timer
                              0,                         // The timer ID is initialised to 0
                              prvTimerCallback );        // The callback function to be used by the timer being created
				     
    // Blink Blue LED: Create the auto-reload, storing the handle to the created timer in xBlueLedReloadTimer.
    xBlueLedReloadTimer = xTimerCreate( "AutoReload LED", BLUE_LED_RELOAD_TIMER_PERIOD,
                              pdTRUE, 0, prvTimerCallback );


    // xWatchdogReloadTimer
    // Watchdog timer update(): Create the auto-reload, storing the handle to the created timer in xWatchdogReloadTimer.
    xWatchdogReloadTimer = xTimerCreate( "AutoReload Watchdog", WATCHDOG_RELOAD_TIMER_PERIOD,
                              pdTRUE, 0, prvTimerCallback );

    // Check FreeRTOS software timers were created.
    if( ( xOneShotTimer != NULL ) && (xBlueLedReloadTimer != NULL) && (xWatchdogReloadTimer != NULL)) {
      // create FreeRTOS tasks
      xTaskCreate(vTaskBlinkDefault, "Blink Task", 128, NULL, 1, NULL);         // Pico defualt LED
	    
      // Create the 'handler' tasks, which are the tasks to which interrupt
      // processing is deferred, and so are the tasks that will be synchronized
      // with the interrupt.  The handler tasks are created with a high priority to
      // ensure they run immediately after the interrupt exits.  In this case a
      // priority of 3 is chosen.
      // UART RX
      xTaskCreate( vUartRxDeferredIntrHandlerTask, "UART RX Handler", 1000, NULL, 3, &xUartRXHandlerTask );
      // PIO FIFO RX
      xTaskCreate( vPioRxDeferredIntrHandlerTask, "PIO FIFO RX Handler", 1000, NULL, 3, &xPioRXHandlerTask );
      // Pushbutton
      xTaskCreate( vPshbttnDeferredIntrHandlerTask, "Pushbutton Handler", 1000, NULL, 3, &xPshbttnHandlerTask );
      
      
      // ****************
      // BLE Tasks
      // ****************
      // Bluetooth Low Energy (BLE)
      //
      //  BTstack Task used to initialize BLE functions and register callbacks
      xTaskCreate( 
          vBTstack_HandlerTask,      // pointer to the task
          "BLE Input Handler",       // task name for kernel awareness debugging
          1000,                      // task stack size   
          NULL,                      // optional task startup argument
          3,                         // initial priority
          &xBTstack_HandlerTask      // optional task handle to create
      );
        
      // Start the timers, using a block time of 0 (no block time).  The
      // scheduler has not been started yet so any block time specified here
      // would be ignored anyway.
      xTimer1Started = xTimerStart( xOneShotTimer, 0 );
      xTimerBlueLedStarted = xTimerStart( xBlueLedReloadTimer, 0 );
      // Watchdog
      xTimerWatchdogStarted = xTimerStart( xWatchdogReloadTimer, 0 );

      // The implementation of xTimerStart() uses the timer command queue, and
      // xTimerStart() will fail if the timer command queue gets full.  The timer
      // service task does not get created until the scheduler is started, so all
      // commands sent to the command queue will stay in the queue until after
      // the scheduler has been started.  Check calls to xTimerStart() passed.
      if( ( xTimer1Started == pdPASS ) && (xTimerBlueLedStarted == pdPASS) && (xTimerWatchdogStarted == pdPASS)) {
        vTaskStartScheduler();   // Start the scheduler
      }
    }
    
   
    // If the scheduler was started then the following line should never be
    // reached because vTaskStartScheduler() will only return if there was not
    // enough FreeRTOS heap memory available to create the Idle and (if configured)
    // Timer tasks.  Heap management, and techniques for trapping heap exhaustion,
    // are described in the book text.
    for( ; ; ) { }

}

// ********************Functions******************************
//
// ***********************************************************

// ********************************
// FreeRTOS Tasks
// ********************************
/*
// B_Bot Commands
void app_set(void) {                   // initialize command array
   command[0].cmnd = "mem";            // memory command
   command[1].cmnd = "fwd";            // vehicle forward command; auto mode
   command[2].cmnd = "stp";            // vehicle stop command; turn off motors command
   command[3].cmnd = "tst";            // Test command
   command[4].cmnd = "epw";            // Enable motor PWM
   command[5].cmnd = "dpw";            // Disable motor PWM
   command[6].cmnd = "son";            // enable 10 uSec HC-SR04 ultrasonic trigger pulse
   command[7].cmnd = "sof";            // disable 10 uSec HC-SR04 ultrasonic trigger pulse
   command[8].cmnd = "trn";            // motor turn delay in 60 mSec increments
   command[9].cmnd = "hlt";            // Vehicle Halt command; stop motors and HC-SR04
   command[10].cmnd = "rgt";           // Manual mode, vehicle Right turn
   command[11].cmnd = "lft";           // Manual mode, vehicle Left turn
   command[12].cmnd = "rev";           // Manual mode, vehicle reverse movement
   command[13].cmnd = "spd";           // configure PWM duty cycle to control speed
*/

// *****************
// BLE Event Handler
// *****************
// Magic happens here: Intersection of B_Bot code, FreeRTOS, and BTstack
// Also, service_implementation.h important for BLE events.
// BLE user input, either commands or text messages; Handler Task
// Use Notification instead of semaphore
// There are 3 possible types of B_Bot BLE events:
// 1) Command (e.g., "fwd", "hlt", "rgt", "lft", "rev", "mem", "trn", "stp", "epw", "dpw", "son", "sof", "spd", "tst")
// 2) Message (e.g., "Hi Al")
// 3) Led Control (e.g., "on", "off") 
//
// User input from client (cell phone), either a command, text message, or Led control.
// Text message and command will be displayed on the SSD1306. Check for the flag blecmdtxt_ptr->is_cltCmd 
// and if it is true we have a command that will be decoded and executed.  
// If it is a text message then it will be displayed on the SSD1306 and nothing has to be done. 
// The blecmdtxt_ptr->is_cltCmd flag is update in service_implementation.h
void vBTstack_HandlerTask( void * pvParameters ) {
    static uint32_t  valid_command;             // valid command indicator
    uint64_t btstack_duration_time = 0;
    static char btstack_duration_str[20];
    uint32_t btstack_tsk_cnt = 0;

    // *******************************
    btstack_memory_init();
 
    l2cap_init(); // Set up L2CAP and register L2CAP with HCI layer
    sm_init(); // setup security manager

#ifdef UART_LOG        
    dbg_print('T');   // debug point
#endif  
  
    // Initialize ATT server, no general read/write callbacks
    // because we'll set one up for each service
    att_server_init(profile_data, NULL, NULL);   

    // Instantiate our custom service handler
    custom_service_server_init();   // moved buffer definitions to severice_implementation.h

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler; // setup callback for events
    hci_add_event_handler(&hci_event_callback_registration); // register callback handler

    // register for ATT event
    att_server_register_packet_handler(packet_handler); // register packet handler

    hci_power_control(HCI_POWER_ON); // turn BLE on
    // *******************************
  
    // The RP2350's PIO handles BTstack's Transport Layer therefore the RP2350 doesn't need
    // a separate FreeRTOS Task to handle data transfers between the RP2350 and the CYW43439.
    // I think that is why the Pico W doesn't need to use a btstack_run_loop_execute() call.
    // The data transfers are done automatically by the PIO.  The RP2350 just has to read the 
    // PIO's Fifo when data from the CYW43439 is ready or write data to the Fifo when sending
    // data to the CYW43439...and then transmitting to the Anroid Phone via BLE protocol.
    // btstack_run_loop_execute();    -> not needed for RP2350 because of PIO

    // Implement this task within an infinite loop.
    for( ; ; ) {       
        // Wait to receive a notification sent directly to this task from custom_service_write_callback()
        // function in service_implementation.h.  The xClearCountOnExit parameter is now pdFALSE, so
        // the task's notification will be decremented when ulTaskNotifyTake() returns having received 
        // a notification.
        if( ulTaskNotifyTake( pdFALSE, portMAX_DELAY ) != 0 ) {
          // To get here the event must have occurred.

          time_matx[btstack_tsk_cnt][2][0] = time_us_64();        // save task start time

#ifdef UART_LOG
          uart_puts(UART_ID, "Task_Handler: command or message from client\n\r");
          // send command to uart         
          uart_puts(UART_ID, "ble_input from client: ");
          uart_puts(UART_ID, blecmdtxt_ptr->ble_input);
          uart_puts(UART_ID, "\n\r");
          // send text to uart
          uart_puts(UART_ID, "client_message from client: ");
          uart_puts(UART_ID, blecmdtxt_ptr->client_message);
          uart_puts(UART_ID, "\n\r");
#endif

          // ************
          // First option: Command
          // ************
          // Decode and execute valid commands
          if (blecmdtxt_ptr->is_cltCmd == true) {             // Do we have a command or text message?
            // Check operator command and execute if valid
            valid_command = cmd_deco (blecmdtxt_ptr->ble_input, TRUE);   //  decode and execute command
            check_ctrl_stack();
            if (valid_command==0) {
                // Not a valid command so assume it is a message to be displayed on UART
                // Message should already be in the client_message[] buffer; done in service_implementation.h
#ifdef UART_LOG 
                uart_puts(UART_ID, "Not a BLE valid command.\r\n");
#endif
            }
            else {
#ifdef UART_LOG 
                uart_puts(UART_ID, "BLE valid command.\r\n");
#endif
            }

            // Prepare to display the command on the SSD1306
            // clear client_message buffer
            memset(blecmdtxt_ptr->client_message, 0, sizeof(blecmdtxt_ptr->client_message)) ;
            // void *memcpy(void *dest, const void *src, size_t n)
            memcpy(blecmdtxt_ptr->client_message, "Cmd: ", 5);
            for (int jj=0; jj<5; jj++) {
                blecmdtxt_ptr->client_message[jj + 5] = blecmdtxt_ptr->ble_input[jj];
            }
            
            blecmdtxt_ptr->is_cltCmd = false;    // get ready for next BLE client event
          }    // end of command 

          // ************
          // Second option: Message
          // ************  
          // Nothing has to be done, the message is ready to be displayed on
          // the SSD1306.
          if (blecmdtxt_ptr->is_cltTxt == true) { 
            blecmdtxt_ptr->is_cltTxt = false;    // get ready for next BLE client event
          }

          // for both command or message, text will be displayed on the SSD1306
#ifdef UART_LOG 
          uart_puts(UART_ID, "Displayed on SSD1306: ");
          uart_puts(UART_ID, blecmdtxt_ptr->client_message);
          uart_puts(UART_ID, "\n\r");       uart_puts(UART_ID, "\n\r");
#endif 


          // ************
          // Third option: Led Control
          // ************
          // Check PCB LED control
          if (blecmdtxt_ptr->is_pcbLed == true) {            // did client press LED on or off

            // Did the user press On or Off
            if ( veh_ptr->led_pcb_on == true ) {
              gpio_put(LED_EX1, 1);
              set_characteristic_c_value( "on" );    // update in service_implementation.h
            } else {
              gpio_put(LED_EX1, 0);
              set_characteristic_c_value( "off" );   // update in service_implementation.h
            }
            
            blecmdtxt_ptr->is_pcbLed = false;    // get ready for next BLE client event

#ifdef UART_LOG
            uart_puts(UART_ID, "PCB LED control pressed.\r\n");
#endif
          }

        time_matx[btstack_tsk_cnt][2][1] = time_us_64();        // save task end time
#ifdef FTOS_LOG
        btstack_duration_time = time_matx[btstack_tsk_cnt][2][1] - time_matx[btstack_tsk_cnt][2][0];
        uart_puts(UART_ID, "BTstack duration in uSec = ");
        sprintf(btstack_duration_str, "%d", btstack_duration_time) ;      // convert to a string 
        uart_puts(UART_ID, btstack_duration_str);
        uart_puts(UART_ID, "\r\n");
        // verify index is working correctly
        uart_puts(UART_ID, "Index = ");
        sprintf(btstack_duration_str, "%d", btstack_tsk_cnt) ;      // convert to a string
        uart_puts(UART_ID, btstack_duration_str);
        uart_puts(UART_ID, "\r\n");
#endif
        btstack_tsk_cnt = (btstack_tsk_cnt + 1) % 8;              // circular buffer index

        }  // end of notification loop
    }      // end of infinite for loop 
}          // end handler task



// Get xBTstack_HandlerTask from main.c and send to service_implementation.h.
// xBTstack_HandlerTask used to communicate between:
// ulTaskNotifyTake() in vBTstack_HandlerTask() in main.c
// and xTaskNotifyGive() in custom_service_write_callback() in service_implementation.h
//TaskHandle_t get_xBLEinput_HandlerTask(void) {
TaskHandle_t get_xBTstack_HandlerTask(void) {
    return xBTstack_HandlerTask;
}

// The blinking LED function; Pico default LED
void vTaskBlinkDefault() {
   uint64_t blink_duration_time = 0;
   static char blink_duration_str[20];
   uint32_t blink_tsk_cnt = 0;
   for (;;) {
      time_matx[blink_tsk_cnt][1][0] = time_us_64();        // save task start time

      // Turn on Pico or Pico W LED
#if defined(PICO_DEFAULT_LED_PIN)
      gpio_put(PICO_DEFAULT_LED_PIN, 1);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
#endif
      vTaskDelay(TaskDEFLTLED_DLY);        // FreeRTOS delay
      // 
      // Turn off Pico or Pico W LED
#if defined(PICO_DEFAULT_LED_PIN)
      gpio_put(PICO_DEFAULT_LED_PIN, 0);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
#endif
      vTaskDelay(TaskDEFLTLED_DLY);        // FreeRTOS delay

      time_matx[blink_tsk_cnt][1][1] = time_us_64();        // save task end time
#ifdef FTOS_LOG
      blink_duration_time = time_matx[blink_tsk_cnt][1][1] - time_matx[blink_tsk_cnt][1][0];
      uart_puts(UART_ID, "Blink duration in uSec = ");
      sprintf(blink_duration_str, "%d", blink_duration_time) ;      // convert to a string 
      uart_puts(UART_ID, blink_duration_str);
      uart_puts(UART_ID, "\r\n");
      // verify index is working correctly
      uart_puts(UART_ID, "Index = ");
      sprintf(blink_duration_str, "%d", blink_tsk_cnt) ;      // convert to a string
      uart_puts(UART_ID, blink_duration_str);
      uart_puts(UART_ID, "\r\n");
#endif
      blink_tsk_cnt = (blink_tsk_cnt + 1) % 8;              // circular buffer index
   }
}


// *********************************
// Software Timer Callback Function
// *********************************
// This function is called everytime the software timer expires. The 
// function determines which of the 2 timers expired and executes code 
// for that software timer.
// Timers:
// 1) Blue blinking LED (auto-reload)
// 2) one-shot for testing  (one-shot)
// 3) Watchdog timer update() to clear watchdog timer
static void prvTimerCallback( TimerHandle_t xTimer ) {
    // The handle of the one-shot timer was stored in xOneShotTimer when the
    // timer was created.  Compare the handle passed into this function with
    // xOneShotTimer to determine if it was the one-shot or auto-reload timer that
    // expired, then output a string to show the time at which the callback was
    // executed.
    // *****************************
    // xTimer for one-shot (just for testing)
    if( xTimer == xOneShotTimer ) {
      // do something here
      uart_puts(UART_ID, "One-shot timer callback executing\n\r");
    }
    // *****************************
    // xTimer for Watchdog timer update()
    else if( xTimer == xWatchdogReloadTimer ) {
      // Clear the watchdog timer
      watchdog_update();

#ifdef WDOG_LOG
      uart_puts(UART_ID, "Watchdog timer callback executing ");
      static char num_str[5];
      sprintf(num_str, "%d", callbck_cnt) ;   // convert to string
      uart_puts(UART_ID, num_str);
      uart_puts(UART_ID, "\r\n");
      callbck_cnt++;
#endif
    }
    // *****************************
    // xTimer for blue blinking LED timer
    // Also used for heartbeat counter and to update SSD1306 display
    else {
      if ((veh_ptr->veh_fwd_active == true) | (pshbttn_fwd == true)) {
//      if ( veh_ptr->veh_fwd_active == true ) {                                  // TODO: I don't think I need pshbttn_fwd
	    gpio_put(BLUE_LED, !gpio_get(BLUE_LED));  // read LED status & toggle LED
	  }
	  else {
	    gpio_put(BLUE_LED, 0);  // // turn off blue LED
	  }

      // Using this reload timer provides a consistent delay.
      // heartbeat;  Increment the counter
      hb_counter += 1 ;
      // Update Counter characteristic (sends to client if notifications enabled)
      set_characteristic_a_value(hb_counter) ;
     
      // If a text message from client display the new message
      if (blecmdtxt_ptr->is_cltCmd == false) {             // Do we have a command or text message?
         memset(blecmdtxt_ptr->ble_input, 0, sizeof(blecmdtxt_ptr->ble_input)) ;   // clear ble_input[] buffer
	     // Display updated client message on SSD1306
	     ssd1306_dsply(FREERTOS_ENABLED, deg_f, blecmdtxt_ptr->client_message, batt_volt_flt);   // new message
      } 
     
      // update temperature every x seconds
      // From: https://github.com/raspberrypi/pico-examples/blob/master/pico_w/bt/standalone/server.c
      if ( hb_counter % 10 == 0) {
         adc_poll();
         // Update Pico Temperature characteristic (sends to client if notifications enabled)
         set_characteristic_d_value(deg_f) ;
	     // Display temperature and client message on SSD1306
	     ssd1306_dsply(FREERTOS_ENABLED, deg_f, blecmdtxt_ptr->client_message, batt_volt_flt);   // use FreeRTOS delay function
      }    
    }  // end of blinking LED else
}



// *********************************
// UART RX Callback Function
// *********************************
// Command UART
// UART RX interrupt handler
// IMPORTANT NOTE: The interrupt is automatically cleared when we read from the data register
//                 or the Fifo is read out to the water-mark. So, no need to clear the interrupt.
static void vUartRxInterruptHandler() {
    uint32_t ii = 0;
    
    // Disable UART RX & TX interrupts and re-enable them in 
    // vUartRxDeferredIntrHandlerTask() function after data is read
    // from the UART RX Fifo. Read RX data in the deferred task to minimize
    // time in this ISR.
    uart_set_irq_enables(UART_ID, false, false);  // disable uart rx & tx interrupts

#ifdef UART_LOG        
    dbg_print('G');   // debug point
#endif    

    BaseType_t xHigherPriorityTaskWoken;
    // The xHigherPriorityTaskWoken parameter must be initialized to
    // pdFALSE as it will get set to pdTRUE inside the interrupt safe
    // API function if a context switch is required.
    xHigherPriorityTaskWoken = pdFALSE;
   
    // Send a notification to the handler task.  The first will unblock the 
    // task, the following 'gives' are to demonstrate that the receiving 
    // task's notification value is being used to latch events - allowing
    // the task to process the events in turn.
    vTaskNotifyGiveFromISR( xUartRXHandlerTask, &xHigherPriorityTaskWoken );
    // The next two 'Give' notifications are for testing only. They work correctly.
//    vTaskNotifyGiveFromISR( xUartRXHandlerTask, &xHigherPriorityTaskWoken );
//    vTaskNotifyGiveFromISR( xUartRXHandlerTask, &xHigherPriorityTaskWoken );
      
}

// ************************
// Command UART
// RX UART Deferred Interrupt Handler Task
// Use Notification instead of semaphore for RX UART interrupt
static void vUartRxDeferredIntrHandlerTask( void * pvParameters ) {
    unsigned char uart_input[80] = "xxx";     // UART input command buffer
    uint32_t valid_command=0;                 // valid command indicator
    
    // Implement this task within an infinite loop.
    for( ; ; ) {       
        // Wait to receive a notification sent directly to this task from the
        // interrupt handler.  The xClearCountOnExit parameter is now pdFALSE, so
        // the task's notification will be decremented when ulTaskNotifyTake()
        // returns having received a notification.
//        if( ulTaskNotifyTake( pdFALSE, xMaxExpectedBlockTime ) != 0 ) {
        if( ulTaskNotifyTake( pdFALSE, portMAX_DELAY ) != 0 ) {
          // To get here the event must have occurred.  Process the event (in this
          // case just print out a message).
          uint32_t ii = 0;
#ifdef UART_LOG        
          dbg_print('D');   // debug point
#endif
          while (uart_is_readable(UART_ID)) {     // do we have RX data?
            uint8_t ch = uart_getc(UART_ID);    // read from UART0 data register
            uart_input[ii] = ch;
            ii++;
            
            // Can we send it back?
            if (uart_is_writable(UART_ID)) {    // is the UART0 data register available
              uart_putc(UART_ID, ch);
            }
          }  // end while loop
        
#ifdef UART_LOG
          uart_puts(UART_ID, "UART RX Deferred Interrupt Handler task - Processing event.\r\n");
#endif
        
          // *****************************************
          // Check operator command and execute if valid
          valid_command = cmd_deco (uart_input, TRUE);   //  decode and execute command
          check_ctrl_stack();                            // remove command if tasks are done

          if (valid_command==0) {
            // assume it is a message from the client and print to SSD1306
#ifdef UART_LOG
            uart_puts(UART_ID, "Not a valid command.\r\n");
#endif
//            cmd_len_uart5 = 0;      // not a valid command
          }
          else {
#ifdef UART_LOG
            uart_puts(UART_ID, "UART RX valid command.\r\n");
#endif
          }
          // Enable UART RX interrupt to wait for next user command
          uart_set_irq_en_wo_timout(UART_ID, true);      // enable only uart rx interrupt
        }  // end of notification loop   
      }    // task infinite loop
  }        // end handler task



// **************************************
// Command UART
// Enable RX UART interrupt without timeout.
// Modified code from:
// /home/arbz/pico/pico-sdk/src/rp2_common/hardware_uart/include/hardware/uart.h
// Does not enable Receive Timeout (RT) interrupt.  User needs time to input commands
// without timeout interrupt.
static inline void uart_set_irq_en_wo_timout(uart_inst_t *uart, bool rx_has_data) {
    // UARTRXINTR (RX) . RX asserts when >=4 characters are in the RX
    // FIFO (for RXIFLSEL=0). RT asserts when there are >=1 characters and no
    // more have been received for 32 bit periods.
    uart_get_hw(uart)->imsc = (bool_to_bit(rx_has_data) << UART_UARTIMSC_RXIM_LSB);
                             
    if (rx_has_data) {
        // Set minimum threshold
        hw_write_masked(&uart_get_hw(uart)->ifls, 0 << UART_UARTIFLS_RXIFLSEL_LSB,
                        UART_UARTIFLS_RXIFLSEL_BITS);
    }
}


// *********************************
// PIO RX FIFO Callback Function
// *********************************
// HC-SR04 Ultrasonic Module; PIO ultrasonic pulse measurements
// PIO RX FIFO interrupt handler
// Interrupt automatically clear when RX FIFO is empty
static void vPioRxInterruptHandler() {
    // Disable PIO RX FIFO interrupt and re-enable them in 
    // vPioRxDeferredIntrHandlerTask() function after data is read
    // from the RX Fifo. Read RX data in the deferred task to minimize
    // time in this ISR.
    hw_clear_bits(&pio0_hw->inte0, 0u << 0);  // disable interrupt
    irq_set_enabled(PIO0_IRQ_0, false);                 // disable irq at NVIC

#ifdef UART_LOG        
    dbg_print('H');   // debug point
#endif
    
    BaseType_t xHigherPriorityTaskWoken2;
    // The xHigherPriorityTaskWoken parameter must be initialized to
    // pdFALSE as it will get set to pdTRUE inside the interrupt safe
    // API function if a context switch is required.
    xHigherPriorityTaskWoken2 = pdFALSE;
    // Send a notification to the handler task.  
    vTaskNotifyGiveFromISR( xPioRXHandlerTask, &xHigherPriorityTaskWoken2 );
} 

// ************************
// HC-SR04 Ultrasonic Module; PIO ultrasonic pulse measurements
// PIO RX FIFO Deferred Interrupt Handler Task
// Use Notification instead of semaphore for PIO RX FIFO interrupt
static void vPioRxDeferredIntrHandlerTask( void * pvParameters ) {
    uint32_t indx=0;
    uint32_t ii=0;
    uint32_t rx_dat_sum=0;
    uint32_t avg_clk_cycles=0;
    uint32_t hc_sr04_echo_rdy = 0;
    uint32_t veh2obs_cm = 0;
    
    // load rx_dat_ary[4] with default values to prevent reverse movement when started
    for (uint32_t jj=0; jj<4; jj++) {
       rx_dat_ary[jj] = DEFLT_RXDATARY;   // defined in defs.h
    }

    // Implement this task within an infinite loop.
    for( ; ; ) {       
        // Wait to receive a notification sent directly to this task from the
        // interrupt handler.  The xClearCountOnExit parameter is now pdFALSE, so
        // the task's notification will be decremented when ulTaskNotifyTake()
        // returns having received a notification.
//        if( ulTaskNotifyTake( pdFALSE, xMaxExpectedBlockTime ) != 0 ) {
        if( ulTaskNotifyTake( pdFALSE, portMAX_DELAY ) != 0 ) {
          // To get here the event must have occurred.  Process the event (in this
          // case just print out a message).

#ifdef UART_LOG        
          uart_puts(UART_ID, "PIO FIFO RX Deferred Interrupt Handler task - Processing event.\r\n");
#endif

          // Read a word of data from a state machine's RX FIFO; interrupt cleared
          uint32_t pio_rx_dat = pio0->rxf[0];
          
          indx = pio_rx_irq_counter & 0x00000003; // save lower two bits
          // store in averaging array
          rx_dat_ary[indx] = pio_rx_dat;
          for (ii=0; ii<4; ii++) {
            rx_dat_sum = rx_dat_sum + rx_dat_ary[ii];
          }
          
          rx_dat_avg = rx_dat_sum/4;
          if (indx=3) {
#ifdef UART_LOG  
            uart_puts(UART_ID, "PIO RX Data Average = ");
            print_int(rx_dat_avg);
            uart_puts(UART_ID, "\r\n");      // debug
#endif
            rx_dat_sum = 0;
            
            // Display average distance.
            // Note: every test for the end of the echo puls takes 2 pio clock ticks,
            //       but changes the 'timer' by only one
            avg_clk_cycles = 2 * rx_dat_avg;
            // set flag to indicate pulse width is ready
            hc_sr04_echo_rdy = 1;
            
            // - the time for 1 pio clock tick (1/150000000 sec for RP2350)
            // - speed of sound in air is about 340 m/s
            // - the sound travels from the HCSR04 to the object and back (twice the distance)
            // we can calculate the distance in cm by multiplying with 0.000136
            veh2obs_cm = (uint32_t)((float)avg_clk_cycles * HCSR04_COEFF);    // HCSR04_COEFF defined in defs.h
#ifdef UART_LOG 
            uart_puts(UART_ID, "cm = ");
            print_int(veh2obs_cm);
            uart_puts(UART_ID, "\r\n");     // debug
#endif
          }

/*
          // To avoid vechicle dither when cm = MINDIS_STOPMTRS, subtract 2 cm
          if ((fsm_ptr->veh_state == VEHBACKWARD_AUTO) || (fsm_ptr->veh_state == VEHSTOP_GOBACK_AUTO)) {
        	// when moving backward to avoid obstacle, force vechicle to move back an
        	// extra inch.
        	fsm_ptr->cm = fsm_ptr->cm - 2.0;
          }
*/
          
          // ********************************
          // Call Vechicle State Machine
          // ********************************
          // Each state executes, then waits for next ultrasonic measurement which
          // is delayed by 60 mSec.  The tick period is 60 mSec.
          // There are two modes of operation: 1) auto obstacle detection, and
          // 2) manual mode where the user has issued a command via Bluetooth LE.
          // But first, must wait for HC-SR04 ultrasonic device to be ready.
          // There is one FSM veh_movmnt_fsm.c.  The main entry to the FSM
		  // is the function update_veh_state() with a parameter veh2obs_cm or
		  // the distance to an obstacle in centimeters.  Before the function is
		  // called the following vehicle state variables must be set:
		  // veh_ptr->veh_fwd_active, veh_ptr->veh_turn_active, and 
		  // veh_ptr->manual_cmd_mode.  The state variables are set in 
		  // cmd_monitor.c.
          if (hc_sr04_echo_rdy==1) {
             update_veh_state(veh2obs_cm);    // Execute from state set above
          }

          // ********************************
          // End of Vechicle State Machine
          // ********************************   
 
          pio_rx_irq_counter++;  // inc to get ready for next sample
        
          // Enable PIO FIFO RX interrupt and wait for next interrupt
          hw_set_bits(&pio0_hw->inte0, 1u << 0);    // enable irq at PIO,SM0_RXNEMPTY
           irq_set_enabled(PIO0_IRQ_0, true);                 // enable irq at NVIC
        }  // end of notification loop   
      }    // task infinite loop
}          // end handler task


// *********************************
// Pushbutton Callback Function
// *********************************
// Pushbutton used as suser input to move the vehicle forward
static void vPshbttnInterruptHandler() { 
     
    // Disable pushbotton interrupt and re-enable them in vPshbttnDeferredIntrHandlerTask() function.
    gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, false, &vPshbttnInterruptHandler);
    
#ifdef UART_LOG        
    dbg_print('K');   // debug point
#endif

    BaseType_t xHigherPriorityTaskWoken3;
    // The xHigherPriorityTaskWoken parameter must be initialized to
    // pdFALSE as it will get set to pdTRUE inside the interrupt safe
    // API function if a context switch is required.
    xHigherPriorityTaskWoken3 = pdFALSE;
    
    // Send a notification to the handler task.  
    vTaskNotifyGiveFromISR( xPshbttnHandlerTask, &xHigherPriorityTaskWoken3 );
}


// ************************
// User input to move vehicle forward and stop
// Pushbutton Deferred Interrupt Handler Task
// Use Notification instead of semaphore for Pushbutton interrupt
static void vPshbttnDeferredIntrHandlerTask( void * pvParameters ) {
   // Implement this task within an infinite loop.
    for( ; ; ) {       
        // Wait to receive a notification sent directly to this task from the
        // interrupt handler.  The xClearCountOnExit parameter is now pdFALSE, so
        // the task's notification will be decremented when ulTaskNotifyTake()
        // returns having received a notification.
        if( ulTaskNotifyTake( pdFALSE, portMAX_DELAY ) != 0 ) {
//          uint32_t pshbtn_command=0;
          // To get here the event must have occurred.  Process the pushbutton event
          // to either start moving forward or stop

//#ifdef UART_LOG
          uart_puts(UART_ID, "Pushbutton Deferred Interrupt Handler task - Processing event.\r\n");
//#endif

          vTaskDelay(5);        // FreeRTOS delay; debounce for 5 mSec
          if (!gpio_get(BUTTON_GPIO)) {  // read pushbutton status after debounce delay
//#ifdef UART_LOG
             uart_puts(UART_ID, "Pushbutton still LOW after debounce delay.\r\n");
//#endif  
          
             // Are we in pushbutton forward mode  
             // if Vehicle is already active, send 'stop' and HC-SR04 off commands       
             if ( pshbttn_fwd == true ) {  
//#ifdef UART_LOG
                uart_puts(UART_ID, "Vehicle is in active mode.\r\n");
//#endif          
                // when pushbutton is pressed and already active, send 'stop' command
                veh_halt_cmd();
                pshbttn_fwd = false;  
                 
             }
             // if Vehicle is not active, send 'forward' command to start moving
             else {                                                    // Vehicle is NOT active
//#ifdef UART_LOG
                uart_puts(UART_ID, "Vehicle is NOT in active mode.\r\n");
//#endif 

                // when pushbutton is pressed move vehicle forward
                char blank = ' ';
                veh_fwd_cmd(&blank);  // this will set default motor speed because a comma is not in command
                pshbttn_fwd = true;
             }     
          }         // end of pushbutton high 
          
          else {
//#ifdef UART_LOG
             uart_puts(UART_ID, "Pushbutton HIGH after debounce delay.\r\n");
//#endif              
          }

          // Enable puahbotton interrupt.
          gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &vPshbttnInterruptHandler);
        }  // end of notification loop         
    }      // end of for loop 
}          // end handler task

// *****************
// Interrput setups
// *****************

// Command UART RX interrupt setup
static void uart_irq_setup(int UART_IRQ) {
    irq_set_exclusive_handler(UART_IRQ, vUartRxInterruptHandler);
    irq_set_enabled(UART_IRQ, true);  // Note: Timer#3 interrupt is enabled??

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, false, false);
    uart_set_irq_en_wo_timout(UART_ID, true);  // rx fifo interrput

    // clear out RX UART FIFO after powerup
    while (uart_is_readable(UART_ID)) {     // check RX Fifo empty flag
        uint8_t ch = uart_getc(UART_ID);    // read from UART0 data register
        uart_putc(UART_ID, ch);
    }

}

// PIO RX FIFO for HC-SR04
// NOTE: Interrupts must be enabled at the peripheral level and at
// NVIC connected to ARM Cortex-M0+ processor.
// If the SM0_RXNEMPTY bit is set in the IRQ0_INTE register, then PIO 
// will raise its first interrupt request line whenever there is a
// character in state machine 0’s RX FIFO.
static void pio_irq_setup(void) {
    // Enable the interrupt for PIO (the PIO outputs 2 irqs)
//    hw_set_bits(&pio0_hw->inte0, 1u << ALARM_NUM);    // enable irq at PIO,SM0 peripheral
//    hw_set_bits(&pio0_hw->inte0, 1u << 8);    // enable irq at PIO,SM0 peripheral
    hw_set_bits(&pio0_hw->inte0, 1u << 0);    // enable irq at PIO,SM0_RXNEMPTY peripheral
    // Set irq handler for PIO,SM0 irq
    // Place the address of pio_irq_handler() in the interrupt vector table. Place
    // the address in the table location associated with ALARM_IRQ.
    irq_set_exclusive_handler(PIO0_IRQ_0, vPioRxInterruptHandler); // update intr vector table VTOR
    // Enable the alarm irq
    irq_set_enabled(PIO0_IRQ_0, true);                 // enable irq at NVIC
    // Enable interrupt in timer peripheral and NVIC connected to ARM Cortex-M0+processor
}

// *****************
// Pico ADC poll for temperature and battery voltage
// *****************
// Raw ADC temperature reading and calculate Pico temperature
void adc_poll(void) {
    // get temperature reading
    temp_reading = get_adc_reading(ADC_CHANNEL_TEMPSENSOR);     // ADC A0 for battery voltage
    
    // The temperature sensor measures the Vbe voltage of a biased bipolar diode, connected to the fifth ADC channel
    // Typically, Vbe = 0.706V at 27 degrees C, with a slope of -1.721mV (0.001721) per degree. 
    deg_c = 27 - (temp_reading - 0.706) / 0.001721;
	// (0°C × 9/5) + 32 = 32°F
	// (0°C x 1.8) + 32 = 32°F
    deg_f = (deg_c * 1.8) + 32;
    
    // Battery voltage measurement
    batt_volt_flt = get_adc_reading(0) * BATT_VOLT_DIV_FACTOR;     // ADC A0 for battery voltage
}

// Get ADC measurement for selected channel
float get_adc_reading(uint32_t chan) {
    adc_select_input(chan);
    uint32_t raw32 = adc_read();
    // Scale raw reading to 16 bit value using a Taylor expansion (for 8 <= bits <= 16)
    uint16_t raw16 = raw32 << (16 - ADC_NUM_BITS) | raw32 >> (2 * ADC_NUM_BITS - 16);
    
    // ref https://github.com/raspberrypi/pico-micropython-examples/blob/master/adc/temperature.py
//    const float adc_conversion_factor = 3.3 / (65535);
//    reading = raw16 * ADC_CONV_FACTOR;   // ADC_CONV_FACTOR defined in defs.h
    return(raw16 * ADC_CONV_FACTOR);   // ADC_CONV_FACTOR defined in defs.h
}


//--------------------------
// Debug Functions
//--------------------------
// write single character to UART
void dbg_print(uint8_t chr) {
    uart_putc(UART_ID, chr);
    uart_putc(UART_ID, '\n');
    uart_putc(UART_ID, '\r');
}

// Print using UART
void dbg_uart_print(float uart_float_num, const char *uart_text) {
    static char adc_str[10];
    
    sprintf(adc_str, "%f", uart_float_num) ;      // convert float to a string
    uart_puts(UART_ID, "\r\n");     // debug   
//    uart_puts(UART_ID, "Battery voltage = ");
    uart_puts(UART_ID, uart_text);
    uart_puts(UART_ID, adc_str);
    uart_puts(UART_ID, "\r\n");     // debug   
}

/*
   // Debug
   static char veh_str[10];
   sprintf(veh_str, "%d", veh_ptr->dutyCycle_primary) ; 
   uart_puts(UART_ID, "veh_ptr->dutyCycle_primary = ");
   uart_puts(UART_ID, veh_str);
   uart_puts(UART_ID, "\r\n");
   
   OR with Al's function
   uart_puts(UART_ID, "Pico temp = ");
   print_float(deg_f);
   uart_puts(UART_ID, "\r\n");     // debug   
   
   // C library functions
   sprintf(volt_str, "%f", batt_volt_flt) ;      // convert float to a string
   snprintf(volt_str, sizeof(volt_str), "%.1f", batt_volt_flt);    // one digit to right of decimal  
   
   // so two ways to send an integer number to UART
   // First: using C compiler functions to convert number to string, then printing string.
         static char num_str[5];
         sprintf(num_str, "%d", (int)ang_result) ;   // if ang_result is a float, convert to string
         uart_puts(UART_ID, num_str);
         uart_puts(UART_ID, "\r\n");
         
   // Second: using ab custom print function
         uart_puts(UART_ID, "\r\n");
         uart_puts(UART_ID, "Turn delay = ");  
         print_int((int)ang_result);   // must #include print_num.h
         uart_puts(UART_ID, "\r\n");  
               
*/

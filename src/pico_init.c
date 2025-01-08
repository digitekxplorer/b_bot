// July 23, 2024
// Support functions for the Combo project that includes:
// 1) Raspberry Pi Pico initialization


// Note: GPIO 0 and 1 are used for UART TX and RX

// ****************************
// Pico
// ****************************

//#include <stdio.h>
//#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/address_mapped.h"
//#include "hardware/gpio.h"
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
#include "defs.h"


// Setup basic pico IO; LEDs and user switches
void pico_io(void) {
//    const uint LED_PIN = PICO_DEFAULT_LED_PIN;  // Pico default LED
    
    // ********************************************
    // Initialize pins for input and output
/*       
    // Default Led output   
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
*/
   
    // Initialize second LED pin
    gpio_init(BLUE_LED);
    gpio_set_dir(BLUE_LED, GPIO_OUT);
    gpio_put(BLUE_LED, 0);
    
    gpio_init(LED_EX1);               // used for BTstack LED "LED Status and Control" characteristic_e
    gpio_set_dir(LED_EX1, GPIO_OUT);
    gpio_put(LED_EX1, 0);

    // Button input
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
//    gpio_pull_down(BUTTON_GPIO);
    // RP2350- E9 correction: use pull-up because of error in RP2350
    gpio_pull_up(BUTTON_GPIO); 
    

    // ********************************************
    // Initialize pins for input and output
    // Used to control TB6612FNG Dual Motor Driver Module

    // Motor#1
    gpio_init(MTR1_AIN1);
	gpio_set_dir(MTR1_AIN1, GPIO_OUT);
	gpio_put(MTR1_AIN1, 1);

    gpio_init(MTR1_AIN2);
	gpio_set_dir(MTR1_AIN2, GPIO_OUT);
	gpio_put(MTR1_AIN2, 0);
	
    // Tell GPIO MTR_PWM allocated to the PWM
    gpio_set_function(MTR1_PWM, GPIO_FUNC_PWM);
    
    // Motor#2
    gpio_init(MTR2_BIN1);
	gpio_set_dir(MTR2_BIN1, GPIO_OUT);
	gpio_put(MTR2_BIN1, 1);

    gpio_init(MTR2_BIN2);
	gpio_set_dir(MTR2_BIN2, GPIO_OUT);
	gpio_put(MTR2_BIN2, 0);
	
    // Tell GPIO MTR_PWM allocated to the PWM
    gpio_set_function(MTR2_PWM, GPIO_FUNC_PWM);
    
    // ****************
    // SSD1306
    // ****************
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);  // (4, 3)
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);  // (5, 3)
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);  
    
    // ********************************************
    // Pins for HC-SR04 are initialized in hcsr04.pio  
}

// Perform Pico or Pico W LED initialization
// Initialize CYW43 for Bluetooth Low Energy (BLE)
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
//    return cyw43_arch_init();
#endif
}

// Pico UART Init
void pico_uart_init(void) {
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
//    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);
    
    // enable CRLF support
//    uart_set_translate_crlf(UART_ID, PICO_UART_DEFAULT_CRLF);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
//    uart_set_fifo_enabled(UART_ID, false);
    uart_set_fifo_enabled(UART_ID, true);    // turn on FIFOs 
    
    // Set FIFO minimum threshold; Interrupt Fifo Level Select (ifls)
//    hw_write_masked(&uart_get_hw(UART_ID)->ifls, 0 << UART_UARTIFLS_RXIFLSEL_LSB, UART_UARTIFLS_RXIFLSEL_BITS);
//    *((io_rw_32 *)(UART0_BASE + UART_UARTIFLS_OFFSET)) = 0x00000008;
//    hw_write_masked(&uart_get_hw(UART_ID)->ifls, 0x00000008;
}

// Initialize Vehicle parameters
// structure: Vehicle parameters
// Used in main.c and monitor.c, declared in defs.h
// manual_cmd_mode=0     vehicle command mode
// veh_state=VEHSTOP_GOFOR_AUTO  vechicle state set forward
// dutyCycle_primary=default   PWM duty cycle for motor speed
// dutyCycle_turn=2      PWM duty cycle for motor speed; used when turning
// veh_turn_dly=26       vechicle turn delay in 60 mSec increments
// active=false          vehicle in active command mode
void veh_stuct_init() {
    veh_ptr->manual_cmd_mode   = 0;
    veh_ptr->dutyCycle_primary = DCYCLE_PRIMARY;   // PWM duty cycle for motor speed; forward and reverse
    veh_ptr->dutyCycle_turn    = DCYCLE_TURN;      // set to turn speed for consistent speed during turns
    veh_ptr->veh_turn_dly      = 26;
    veh_ptr->active            = false;
    
    // structure: Vehicle Movement FSM parameters
    // Initialize Vehicle Movement FSM parameters
    // Used in main.c and veh_movmnt_fsm.c, declared in defs.h
    // veh_state=VEHSTOP_GOFOR_AUTO    vechicle state set forward
    // cm = MINDIS_STOPMTRS            start with 10.0 to allow motors to start
    //
    fsm_ptr->veh_state         = VEHSTOP_GOFOR_AUTO;
    fsm_ptr->cm                = MINDIS_STOPMTRS + 5;
    
    // char array of client message displayed on SSD1306
    memcpy(blecmdtxt_ptr->client_message, "Hi Alfredo", 10) ;
}


// **************************************
// Print Welcome message
void print_welcome(uint32_t sec) {
    // Add several seconds delay to allow USB port to be set
    sleep_ms(1000);
    uart_puts(UART_ID, "\n\rDelay");
    for(int ii=0; ii< sec; ii=ii+1) {
        uart_puts(UART_ID, ".");
//        printf("Delay %d\n", ii);
        sleep_ms(1000);
    }
    uart_puts(UART_ID, "\n\r");
    
    // Welcome Message
    uart_puts(UART_ID, "\nHello, B_Bot with FreeRTOS and BLE\n\r");
}

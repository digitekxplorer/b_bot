// Aug 9, 2024
// Printing functions for integer and floating point numbers
// using uart_putc().  Used to replace printing functions in
// printf().

/*
For Raspberry Pi with a 32-bit processor
From:
https://raspberry-projects.com/pi/programming-in-c/memory/variables

Name	            C++	        Bytes	Range
bool		                    1	    true or false
signed char 	    int8_t	    1	    -128 to 127
unsigned char	    uint8_t	    1	    0 to 255
short int	        int16_t	    2	    -32768 to 32767
unsigned short int	uint16_t	2	    0 to 65535
int	                int32_t	    4	    -2147483648 to 2147483647
unsigned int	    uint32_t	4	    0 to 4294967295
long int	        int32_t	    4	    -2147483648 to 2147483647
unsigned long int	uint32_t	4	    0 to 4294967295
long long	        int64_t	    8	    −9,223,372,036,854,775,808 to 9,223,372,036,854,775,807
unsigned long long	uint64_t	8	    0 to 18,446,744,073,709,551,615
float		                    4	    +/- 3.4e +/- 38 (~7 digits)
double		                    8	    +/- 1.7e +/- 308 (~15 digits)
long double		                8	    +/- 1.7e +/- 308 (~15 digits)
wchar_t		                    2 or 4	1 wide character
*/


#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define UART_ID    uart0
#define BAUD_RATE  115200
#define DATA_BITS  8
#define STOP_BITS  1
#define PARITY     UART_PARITY_NONE

#define BUF_LEN    7                 // integers up to 7 digits

/*
// UART Pins
const uint uart_tx_pin = 0; // GPIO 0 (UART0_TX)
const uint uart_rx_pin = 1; // GPIO 1 (UART0_RX)

// Initialize UART
void uart_init_ab(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(uart_rx_pin, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
}
*/

// integer to ASCII
uint32_t int2ascii(uint32_t value, char* buffer) {
  uint32_t temp=0;
  uint32_t ii=0;
  
  while (value > 0) {  
//  for(ii=0; ii<3; ii++) {
    temp = value % 10;
    buffer[ii] = temp + '0';
    value /= 10;
    ii++;
  }
  return ii;
}

// Reverse buffer order
void reverse_buffer(char* buffer, uint32_t length) {
    uint32_t start = 0;
    uint32_t end = length - 1;
    char temp;

    while (start < end) {
        // Swap characters at start and end indices
        temp = buffer[start];
        buffer[start] = buffer[end];
        buffer[end] = temp;
        // Move towards the center
        start++;
        end--;
    }
}

// Print integer
//void uart_put_int(int value) {
void print_int(uint32_t value) {
    char buffer[BUF_LEN]; // max 4 digits for 32-bit int + null terminator
    uint32_t num_of_chars=0;
    num_of_chars = int2ascii(value, buffer); // convert int to characters stored in buffer
//    reverse_buffer(buffer, 2);
    reverse_buffer(buffer, num_of_chars);
//    for (int i = 0; buffer[i] != '\0'; i++) {
//    for (uint32_t i = 0; i<2 ; i++) {
    for (uint32_t i = 0; i<num_of_chars ; i++) {
        uart_putc(UART_ID, buffer[i]);
    }
}


// Convert to ASCII and print single integer digit
void print_digit(uint32_t value) {
    char value_char = 0;
    if (value>=0 && value<=9) {
      value_char = (char) value + '0';  // convert to ASCII
      uart_putc(UART_ID, value_char);
    }
    else {
      uart_putc(UART_ID, 'X');          // error
    }
}

// Convert float to integer
uint32_t floatToInt(float num) {
    return (uint32_t)num;
}

// Get fractional part of a float
float fractionalPart(float num) {
    return num - (uint32_t)num;
}

// Print floating point number.
// The float point number is partitioned into the whole num and fractional num.
// Prints only one digit of the fractional part.
void print_float(float flt_val) {
    uint32_t int_part = 0;
    float fract_part =0.0;
    uint32_t first_dec = 0;
    int_part = floatToInt(flt_val);       // convert float to integer
    fract_part = fractionalPart(flt_val); // get fractional part of float num
    first_dec = (uint32_t) 10.0*fract_part;            // get first digit after decimal pt, 1/10th value
    // print each part of number individually with point between them
    print_int(int_part);                  // print integer 
    uart_putc(UART_ID, '.');              // print decimal point 
    print_digit(first_dec);               // Convert to ASCII and print single integer digit
    uart_puts(UART_ID, "\n\r");
}

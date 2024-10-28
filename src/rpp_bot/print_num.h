// Aug 9, 2024
// Printing functions for interger and floating point numbers
// using uart_putc().  Used to replace printing functions in
// printf().

// integer to ASCII
uint32_t int2ascii(uint32_t value, char* buffer);

// Reverse buffer order
void reverse_buffer(char* buffer, uint32_t length);

// Print integer
void print_int(uint32_t value);

// Convert to ASCII and print single integer digit
void print_digit(uint32_t value);

// Convert float to integer
uint32_t floatToInt(float num);

// Get fractional part of a float
float fractionalPart(float num);

// Print floating point number.
void print_float(float flt_val);


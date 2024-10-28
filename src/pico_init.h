// July 23, 2024
// Support functions for the Combo project that includes:
// 1) Raspberry Pi Pico initialization

// ****************************
// Pico
// ****************************

// Setup basic Pico IO; LEDs and user switches
void pico_io(void);

// Pico UART Init
void pico_uart_init(void);

// Perform Pico or Pico W LED initialisation
int pico_led_init(void);

// Initialize Vehicle parameters
void veh_stuct_init();

// Print Welcome message
void print_welcome(uint32_t sec);

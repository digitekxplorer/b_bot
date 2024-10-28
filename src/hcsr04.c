// Aug 6, 2024

#include "pico/stdlib.h"
//#include "/home/arbz/pico/pico-sdk/src/rp2_common/hardware_pwm/include/hardware/pwm.h"
#include "hardware/pio.h"

#include "defs.h"
// our PIO assembly header
#include "hcsr04.pio.h"

// Pico PIO setup
uint32_t hcsr04_init(void) {
    // Choose which PIO instance to use (there are two instances)
    PIO pio = pio0;
    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    uint offset = pio_add_program(pio, &hcsr04_program);
    
    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    uint sm = pio_claim_unused_sm(pio, true);
    
}


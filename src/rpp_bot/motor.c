// July 29, 2024
// B-Bot Motor Controls

// Motor Driver: TB6612FNG
// Output PWM signals on pin 6
// System clock = 125MHz
// Use 1KHz clock for motors
// From RP2040 Datasheet; section 4.5.2.6 Config PWM Period
// Period = (TOP +1) x (CSR_PH_CORRECT+1) + (DIV_INT+Div_Frac/16)
// For our application: CSR_PH_CORRECT=0; DIV_INT=1; Div_Frac=0
// So, Period = TOP+1
// fpwm = fsys/Period => Period = fsys/fpwm
// A typical fpwm is from 20KHz to 50KHz; assume 25kHz
// 125MHz/25000 = 5000 counts per period
//
// TB6612FNG
// IN1  IN2    Definition
//  0    0     Stop
//  0    1     Reverse
//  1    0     Forward
//  1    1     Brake

#include "pico/stdlib.h"
//#include "/home/arbz/pico/pico-sdk/src/rp2_common/hardware_pwm/include/hardware/pwm.h"
#include "hardware/pwm.h"

#include "../defs.h"

// Motor slice number struct defined in defs.h.
// Used a struct to store motor slice number to allow us to use the
// struct in other files (i.e. monitor.c)

// complete Motor PWM output setup
void motor_init(void) {
    // initialize slice num struct; defined in defs.h
    slnum_ptr->slice_num1 = 0;
    slnum_ptr->slice_num2 = 0;
    // Find out which PWM slice is connected to GPIO MTR1_PWM and MTR2_PWM
    slnum_ptr->slice_num1 = pwm_gpio_to_slice_num(MTR1_PWM);
    slnum_ptr->slice_num2 = pwm_gpio_to_slice_num(MTR2_PWM);
    
	// Turn off TB6612FNG Dual Motor Drivers
	// Motor#1
	gpio_put(MTR1_AIN1, 0);
	gpio_put(MTR1_AIN2, 0);
	// Motor#2
	gpio_put(MTR2_BIN1, 0);
	gpio_put(MTR2_BIN2, 0);

    // Set period of PERIOD cycles (0 to PERIOD inclusive)
    pwm_set_wrap(slnum_ptr->slice_num1, PERIOD);      // mtr1
    pwm_set_wrap(slnum_ptr->slice_num2, PERIOD);      // mtr2
    // Set channel A output high for DUTY_CYC_A cycles before dropping
    // param slice_num PWM slice number
    // param chan Which channel to update. 0 for A, 1 for B.
    // param level new level for the selected output
    // static inline void pwm_set_chan_level(uint slice_num, uint chan, uint16_t level)
    pwm_set_chan_level(slnum_ptr->slice_num1, MTR1_PWM_CHAN, DUTY_CYC_A);   // mtr1
//    pwm_set_chan_level(slnum_ptr->slice_num2, PWM_CHAN_A, DUTY_CYC_B);   // mtr2
    pwm_set_chan_level(slnum_ptr->slice_num2, MTR2_PWM_CHAN, DUTY_CYC_B);   // mtr2
    // Set initial B output high for DUTY_CYC_B cycles before dropping
//    pwm_set_chan_level(slice_num, PWM_CHAN_B, DUTY_CYC_B);
    // Set the PWM running
    pwm_set_enabled(slnum_ptr->slice_num1, true);
    pwm_set_enabled(slnum_ptr->slice_num2, true);
}

// Clear AIN1&2 and BIN1&2 to have motors start in off state
void motors_off(void) {
	// Turn off TB6612FNG Dual Motor Drivers
	// Motor#1
	gpio_put(MTR1_AIN1, 0);
	gpio_put(MTR1_AIN2, 0);
	// Motor#2
	gpio_put(MTR2_BIN1, 0);
	gpio_put(MTR2_BIN2, 0);
}


// Enable PWM for both motors
//void motor_pwm_enable(uint32_t slice_num) {
void motor_pwm_enable(void) {
    pwm_set_enabled(slnum_ptr->slice_num1, true);      // enable mtr1 PWM
    pwm_set_enabled(slnum_ptr->slice_num2, true);      // enable mtr2 PWM
}


// Turn OFF PWM for both motors
void motor_pwm_disable(void) {
    pwm_set_enabled(slnum_ptr->slice_num1, false);      // disable mtr1 PWM
    pwm_set_enabled(slnum_ptr->slice_num2, false);      // disable mtr1 PWM
}


// **************************************************************************
// **************************************************************************
// Pin configuration for TB6612FNG Dual Motor Driver Module
// Motor#1: GP8=AIN1, GP7=AIN2
// Motor#2: GP?=BIN1, GP?=BIN2
// IN1    IN2    PWM    /STBY    OUT1    OUT2    Mode
// ----------------------------------------------------------
// H      H      H/L    H        L       L       Short Brake
// L      H      H      H        L       H       CCW
// L      H      L      H        L       L       Short Brake
// H      L      H      H        H       L       CW
// H      L      L      H        L       L       Short Brake
//                               OFF
// L      L      H      H        High Impedance  STOP
//                               OFF
// H/L    H/L    H/L    L        High Impedance  Standby
//
// Motors CCW to move forward
void motor_forward(void) {
	   // Set TB6612FNG Dual Motor Driver to CCW
	   // Motor#1
	   gpio_put(MTR1_AIN1, 1);
	   gpio_put(MTR1_AIN2, 0);
	   // Motor#2
	   gpio_put(MTR2_BIN1, 1);
	   gpio_put(MTR2_BIN2, 0);
}

// Motors CW to move backward
void motor_backward(void) {
	   // Set TB6612FNG Dual Motor Driver to CW
	   // Motor#1
	   gpio_put(MTR1_AIN1, 0);
	   gpio_put(MTR1_AIN2, 1);
	   // Motor#2
	   gpio_put(MTR2_BIN1, 0);
	   gpio_put(MTR2_BIN2, 1);
}

// Motors turn Right
// Motor#1 CCW
// Motor#2 CW
void motor_turnRight(void) {
	   // Set TB6612FNG Dual Motor Driver
	   // Motor#1 CCW
	   gpio_put(MTR1_AIN1, 0);
	   gpio_put(MTR1_AIN2, 1);
	   // Motor#2 CW
	   gpio_put(MTR2_BIN1, 1);
	   gpio_put(MTR2_BIN2, 0);
}

// Motors turn Left
// Motor#1 CW
// Motor#2 CCW
void motor_turnLeft(void) {
	   // Set TB6612FNG Dual Motor Driver
	   // Motor#1 CW
	   gpio_put(MTR1_AIN1, 1);
	   gpio_put(MTR1_AIN2, 0);
	   // Motor#2 CCW
	   gpio_put(MTR2_BIN1, 0);
	   gpio_put(MTR2_BIN2, 1);
}


// Set PWM duty cycle
//uint32_t motor_set_dutyCycle(uint32_t dutyCycle, uint32_t multiplier) {
uint32_t motor_set_dutyCycle(uint32_t dutycycle) {
    // Set motor speed by adjusting the PWM duty cycle.  Full speed is 5000, so multiply operator
    // input (1 to 10) by 500 to calulate the desired speed.
    pwm_set_chan_level(slnum_ptr->slice_num1, MTR1_PWM_CHAN, (uint16_t)(MTRDCYCLEMULTIPLIER1 * dutycycle));  // mtr1
//    pwm_set_chan_level(slnum_ptr->slice_num2, MTR1_PWM_CHAN, (uint16_t)(MTRDCYCLEMULTIPLIER2 * dutycycle));  // mtr2
    pwm_set_chan_level(slnum_ptr->slice_num2, MTR2_PWM_CHAN, (uint16_t)(MTRDCYCLEMULTIPLIER2 * dutycycle));  // mtr2
   return (1);
   }

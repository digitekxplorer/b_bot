/*
 * monitor.h
 *
 *  Created on: Oct 19, 2014
 *      Author: Al Baeza
 */

#ifndef CMDMONITOR_H_
#define CMDMONITOR_H_

#define  TRUE    1
#define  FALSE   0


// Support Code
unsigned int cmd_deco (unsigned char operator_input[], unsigned int exe_cmd);         // command decode
//void cmd_deco (unsigned char operator_input[]);         // command decode
unsigned int rep_cmd_deco (unsigned char operator_input[],      // repeat command check
		                   unsigned char operator_repeat[]);

void app_set();                  // application set
void monitor_init();             // monitor initialization
void check_ctrl_stack();         // check control stack

// Declare command array
int memory_cmd(char *cp);        // microcontroller internal memory
int veh_fwd_cmd(char *cp);       // vehicle forward command, turn on motors; auto mode
int veh_stop_cmd();              // vehicle stop command, turn off motors
int tst_cmd();                   // Test command
int en_pwm_cmd();                // Enable motor PWM
int dis_pwm_cmd();               // Disable motor PWM
int digitize_cmd();              // A/D digitizing
int set_pwm_mult_cmd(char *cp);  // right side motor PWM multiplier
int hc_sr04_enable_cmd();        // enable 10 usec HC-SR04 ultrasonic trigger pulse
int hc_sr04_disable_cmd();       // disable 10 usec HC-SR04 ultrasonic trigger pulse
int veh_halt_cmd();              // Vehicle Halt command; stop motors and HC-SR04

int veh_turn_dly_cmd(char *cp);  // Vehicle turn delay; adjust the delay in 60 mSec increments
int veh_rTrn_cmd();              // vehicle right turn; manual mode
int veh_lTrn_cmd();              // vehicle left turn; manual mode
int veh_rev_cmd();               // vehicle reverse movement; manual mode
int veh_speed_cmd(char *cp_input); // configure PWM duty cycle to control vehicle speed

// Tasks
int memory_addr (unsigned int st_address);


#endif /* MONITOR_H_ */

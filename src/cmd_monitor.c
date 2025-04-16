/**
 * July 26, 2024
 * Command monitor.
 *
 *  Originally created on Oct 19, 2014 for the A_Bot project
 *      Author: Al Baeza
 *      These functions are part of the general purpose control program.
 *      The program is based on Rick Brown's "C Function Pointers Let You
 *      Build Generic Control Systems".
 */
 
 
 
// Include necessary libraries
#include "pico/stdlib.h"
#include <stdio.h> 
#include <string.h>
#include <ctype.h>
#include "hardware/pwm.h"
#include "hardware/pio.h"

#include "cmd_monitor.h"
#include "motor.h"
#include "defs.h"

#include "print_num.h"

// Number of Tasks in stack
#define   NUM_CMDS  14    // Number of operator commnads
#define   TASK_SIZE  8    // Number of tasks per command

#define   SERBUFSIZE 1024


// *******************
// Monitor variables
// *******************
int tsk_stkptr;             // task stack index
int tskLp_indx;             // task loop index
int func_parm;              // function parameter

// ***************************************************************************
// Structure Definitions
// ***************************************************************************

// Control Transfer Stack
struct                     // structure array of size TASK_SIZE
{
  int (*func) ();          // declare a function pointer
  unsigned long int parm;  // and associated parameter
} task[TASK_SIZE];

// Operator Control Transfer Stack
struct
{
  int (*func) ();              // function pointer
  char *cmnd;                  // pointer to command text
} command[NUM_CMDS];


// Support Code
unsigned int cmd_deco (unsigned char operator_input[], unsigned int exe_cmd);         // command decode
//void cmd_deco (unsigned char operator_input[]);         // command decode
unsigned int rep_cmd_deco (unsigned char operator_input[],      // repeat command check
		                   unsigned char operator_repeat[]);

void app_set();                  // application set
void monitor_init();             // monitor initialization
void check_ctrl_stack();         // check control stack


// Routine to initialize application.
// Note: Remember to update NUM_CMDS in defs.h when user commands are added.  The number
// of commands must match the command index number plus one.
void app_set(void) {                   // initialize command array
   command[0].func = memory_cmd;       // memory command
   command[0].cmnd = "mem";
   command[1].func = veh_fwd_cmd;      // vehicle forward command; auto mode
   command[1].cmnd = "fwd";
   command[2].func = veh_stop_cmd;     // vehicle stop command; turn off motors command TODO: remove, debug only
   command[2].cmnd = "stp";
   command[3].func = veh_rev_cmd;      // Manual mode, vehicle reverse movement TODO: remove, debug only
   command[3].cmnd = "rev";
   command[4].func = en_pwm_cmd;       // Enable motor PWM
   command[4].cmnd = "epw";
   command[5].func = dis_pwm_cmd;      // Disable motor PWM
   command[5].cmnd = "dpw";
   command[6].func = hc_sr04_enable_cmd; // enable 10 uSec HC-SR04 ultrasonic trigger pulse
   command[6].cmnd = "son";              // sonic trigger pulse
   command[7].func = hc_sr04_disable_cmd;// disable ultrasonic trigger pulse
   command[7].cmnd = "sof"; 
   command[8].func = veh_turn_dly_cmd;   // motor turn delay in 60 mSec
   command[8].cmnd = "trn";
   command[9].func = veh_halt_cmd;      // Vehicle Halt command; stop motors and HC-SR04
   command[9].cmnd = "hlt";
 
   command[10].func = veh_rTrn_cmd;       // Manual mode, vehicle Right turn
   command[10].cmnd = "rgt";
   command[11].func = veh_lTrn_cmd;       // Manual mode, vehicle Left turn
   command[11].cmnd = "lft";
   command[12].func = veh_bwd_cmd;       // Manual mode, vehicle backward movement
   command[12].cmnd = "bwd";
   command[13].func = veh_speed_cmd;     // configure PWM duty cycle to control speed
   command[13].cmnd = "spd";
   
/*
   command[4].func = set_pwm_mult_cmd; // adjustment for PWM multiplier to calibrate for straight movement
   command[4].cmnd = "srt";            // adjust right side motor PWM multiplier
*/
   }
   
// *****************************************
//    Application code
// *****************************************

// The structure of the control program is as follows:
//   Commands are used to place the desired action on the stack.
//   Commands are broken down into tasks.

// Tasks within the commands
int memory_zero(), memory_addr();            // functions in Memory command

// ************************
//   SUPPORT CODE
// ************************

//   Routine to detect operator entered commands.
unsigned int cmd_deco (unsigned char operator_input[], unsigned int exe_cmd)
//void cmd_deco (unsigned char operator_input[])
  {
//	unsigned char *cp1, *cp2;
	unsigned char *cp1;
	char *cp2;            // Note: cp2 must be char since cmnd string are also char
	int i;
	unsigned int command_found;

//    command_found = FALSE;
    command_found = false;
	for (i=0; i<NUM_CMDS; i++)    // determine if command entered
		{
		cp1 = operator_input;     // command input buffer
		cp2 = command[i].cmnd;    // possible command string
				
		do
			{
			if (*(cp1++) != *(cp2++))   // test for command input; ab 5/25/2024
				break;                  // not this command

			if (!*cp2)                  // end of string, we have command
				{
				if (exe_cmd > 0) {
				  (*command[i].func) (cp1);  // ****** transfer control to this command  *********
				}
				i = 999;                   // abort for loop
				command_found = true;
				break;
				}
			}
		while (*cp1);    // until end of input string or command detected
		}
	if (command_found == false)
	   {
#ifdef UART_LOG
//       printf("Invalid Command.\n");
       uart_puts(UART_ID, "Invalid Command..\r\n");
#endif
	   }
	return (command_found);
	}


//   Routine to check for operator repeat command.
unsigned int rep_cmd_deco (unsigned char operator_input[], unsigned char operator_repeat[])
	{
	int i;
	unsigned int rep_cmd_found;
    rep_cmd_found = false;

    for (i=0; i<3; i++)      // check first three characters
       {
       if (operator_input[i] == operator_repeat[i])   // repeat command?
          rep_cmd_found = true;
       else
          {
          rep_cmd_found = false;
          i=4;                     // abort loop
          }
       }

    return (rep_cmd_found);
	}



// Rountine to place a function on the control stack.
void push_task (int (*func) (), int parm)
   {
   if (tsk_stkptr >= TASK_SIZE) {
#ifdef UART_LOG
//	  UART0_OutString(mess06);
//	  UART0_sendCRLF();
#endif
	  }
   else {
	  task[tsk_stkptr].func = func;     // place new entry
	  task[tsk_stkptr++].parm = parm;   // and parameter on stack
	  }
   }

// Routine to remove a function from the control stack.
void tsk_unstack (int *func_indx)
   {
   int j, k;

   k = *func_indx;         // index of function to remove
   for (j=k; j<tsk_stkptr; j++, k++) {
      task[k].func = task[j].func;   // squeeze function out of stack
   	  task[k].parm = task[j].parm;
      }
   tsk_stkptr--;                  // reduce stack size
   (*func_indx)--;                        // reduce index of callers loop
   }

// Routine to check control stack
void check_ctrl_stack(void) {
   for (tskLp_indx=0; tskLp_indx<tsk_stkptr; tskLp_indx++)  // check the control stack
      {
      func_parm = (*task[tskLp_indx].func) (task[tskLp_indx].parm);
      task[tskLp_indx].parm =func_parm;
      if (!func_parm)
         tsk_unstack (&tskLp_indx);
      }
   }

// Monitor initialization
void monitor_init(void) {
	tsk_stkptr = 0;             // clear task stack pointer
}

// Function to convert char to a hex integer
unsigned int char_to_hex_int (char parameter)
   {
   unsigned int result = 0;

   if (parameter >= '0' && parameter <= '9')
      result = parameter - '0';
   else
      result = parameter - 'A' + 10;

   return (result);
   }


// Function to convert string to a hex integer
unsigned int string_to_hex_int (char *string)
   {
   int i;
   unsigned int integer_value, result = 0;

   for (i=0; (*string >= '0' && *string <= '9') ||
             (*string >= 'A' && *string <= 'F'); i++)
      {
      if (*string >= '0' && *string <= '9')
         integer_value = *string - '0';
      else
         integer_value = *string - 'A' + 10;
      result = result * 16 + integer_value;
      *string++;
      }
   return (result);
   }
   
// Function to convert string to a decimal integer
unsigned int string_to_dec_int (char *string)
   {
   int i;
   unsigned int integer_value, result = 0;

   for (i=0; (*string >= '0' && *string <= '9'); i++)
      {
      integer_value = *string - '0';
      result = result * 10 + integer_value;
      *string++;
      }
   return (result);
   }

// ************************************
//         COMMANDS
// ************************************

// Routine to process MEMORY command.
int memory_cmd(char *cp)           // first function of Memory command
                                   // *cp is pointer to next character in the command string
   {
   unsigned int byte_address;

   if (*cp == ',')
      {
      *cp++;                         // next character in string
      byte_address = string_to_hex_int (cp); //  convert to integer
      push_task (memory_addr, byte_address); // memory dump with starting addr
      return (1);
      }
   else
      {
      push_task (memory_zero, 0);    // memory dump starting at zero
      return (2);
      }
   }

// ********************************************
// Vehicle forward using PWM to control motors
int veh_fwd_cmd(char *cp_input) {
   unsigned char dutyCycle_char = '0';
   
         // debug
/*
         uart_puts(UART_ID, "Inside veh_fwd_cmd().\r\n");
         uart_puts(UART_ID, "Turn delay = ");
         
         static char num_str[5];
         sprintf(num_str, "%d", veh_ptr->veh_turn_dly) ; 
         uart_puts(UART_ID, num_str);
         uart_puts(UART_ID, "\r\n");  
         print_int(veh_ptr->veh_turn_dly);
         uart_puts(UART_ID, "\r\n"); 
*/
         // end debug



   if (*cp_input != ',') {
	   // set to default if no duty cycle value included
	   // dutyCycle=(1 to 20); multiplier=250
	   motor_set_dutyCycle(veh_ptr->dutyCycle_primary);      // set motor speeds; defined in defs.h and set in main.c
   }
   else {
       *cp_input++;                                            // inc to next parameter
       dutyCycle_char = *cp_input;
       veh_ptr->dutyCycle_primary = char_to_hex_int (dutyCycle_char);   // convert speed duty cycle to integer
       // check new speed
	   if (veh_ptr->dutyCycle_primary>1 && veh_ptr->dutyCycle_primary<DCYCLE_MAX) {
	    	   motor_set_dutyCycle(veh_ptr->dutyCycle_primary);    // set motor speeds
	   }
	   else {
			   // set to default speed if no duty cycle value included
	    	   motor_set_dutyCycle(DCYCLE_PRIMARY);    // set motor speeds
	   }     
//       motor_set_dutyCycle(veh_ptr->dutyCycle_primary);       // set motor speeds
   }

   // Enable HC-SR04 trigger pulse
   hc_sr04_enable_cmd();
   veh_ptr->manual_cmd_mode = 0;             // set auto obstacle detection mode using HC-SR04

   // Set TB6612FNG Dual Motor Driver to CCW to move forward
   motor_forward();
   veh_ptr->active = true;                   // vehicle in active mode

   // After PWM duty cycle for both motors is set, enable PWM
   motor_pwm_enable();


   return (1);
   }
   
   
// Vehicle motors Stop command
int veh_stop_cmd() {
   // Motor#1 and Motor#2
   motors_off();      // motor.c
   veh_ptr->active = false;         // vehicle is not in active mode
   return (1);
   }

// Vehicle Halt command; stop motors and HC-SR04
int veh_halt_cmd() {
   // Motor#1 and Motor#2
   motors_off();      // motor.c

   // also stop HC-SR04
   PIO pio = pio0;
   uint sm = 0;
   pio_sm_put_blocking(pio, sm, 2*SM_START_CMD);  // stop cmd value = 2*5  
   veh_ptr->active = false;         // vehicle is not in active mode
   return (1);
   }

// Vehicle Reverse command
int veh_rev_cmd() {
   // Motor#1 and Motor#2
   motor_backward();     // motor.c
   return (1);
   }

// Enable motor PWM
// Motor slice number struct defined in defs.h and intialized in motor.c
int en_pwm_cmd() {
    motor_pwm_enable();   // motor.c
   return (1);
   }
   
// Disable motor PWM
// Motor slice number struct defined in defs.h and intialized in motor.c
int dis_pwm_cmd() {
   motor_pwm_disable();   // motor.c
   return (1);
   }
   
// Enable HC-SR04 trigger pulses
// Send command to PIO TX FIFO to start state machine.
int hc_sr04_enable_cmd() {
   PIO pio = pio0;
   uint sm = 0;
   pio_sm_put_blocking(pio, sm, SM_START_CMD);  // start cmd value = 5
   return (1);
   }
   
// Disable HC-SR04 trigger pulses
// Send command to PIO TX FIFO to stop state machine.
// The stop command value is twice the start value (used as a check)
int hc_sr04_disable_cmd() {
   PIO pio = pio0;
   uint sm = 0;
   pio_sm_put_blocking(pio, sm, 2*SM_START_CMD);  // stop cmd value = 2*5
   return (1);
   }
   
// ********************************************
// TODO: debug code
// Vehicle turn delay
// Example command: trn,090
// Must be three digits after comma.
// Routine to adjust the delay in 60 mSec increments during a turn.
// Adjustment must be decimal.
// Motor PWM or duty cycle is proportional to speed.
// The vehicle movement FSM runs at 60 mSec increments, so we count the number of 60 mSec
// increments needed to turn the vechicle the desired amount. In other words, the turn
// resolution is 60 mSec.
// For TT motors it takes about 7 60 mSec increments to turn 90 degrees (TURNDLY_90DEG = 7).
// The turn duty cycle (or speed) is set to 5 (DCYCLE_TURN = 5).
// So if we want to turn 90 degress, the turn delay should be set to 7. For 180, it should be 14.
int veh_turn_dly_cmd(char *cp) {   // multiplier adjustment
                                   // *cp is pointer to next character in the command string
//   int turn_angle = 26;
   int turn_angle = TURN_ANGLE;    // define in defs.h
   float ang_dly = TURNDLY_90DEG;
   unsigned char char_hndrds, char_tens, char_ones;
//   int parsed_number = 0;
   // if a comma is included in the command then we have a decimal number following the comma
   if (*cp == ',') {
      *cp++;                                   // next character in string
      char_hndrds = *cp++;                     // store hundreds position value
      char_tens = *cp++;                       // store tens position value
      char_ones = *cp;                         // store ones position value
      
      if (isdigit(char_ones) && isdigit(char_tens) && isdigit(char_hndrds)) {
         turn_angle = (char_hndrds - '0') * 100 + (char_tens - '0') * 10 + (char_ones - '0');   // convert to integer
//         ang_dly = 0.0778 * (float)turn_angle;  // delay = 7/90 * turn_angle
         ang_dly = TURN_COEFF * (float)turn_angle;  // delay = TURNDLY_90DEG/90 * turn_angle
         veh_ptr->veh_turn_dly = (int)ang_dly; // used in veh_movmnt_fsm.c

         // debug
/*
         uart_puts(UART_ID, "Inside veh_turn_dly_cmd().\r\n");
         uart_puts(UART_ID, "Turn delay = ");
         
         static char num_str[5];
         sprintf(num_str, "%d", veh_ptr->veh_turn_dly) ; 
         uart_puts(UART_ID, num_str);
         uart_puts(UART_ID, "\r\n");  
         print_int(veh_ptr->veh_turn_dly);
         uart_puts(UART_ID, "\r\n"); 
*/
         // end debug
         
         return (1);
      } else {
         veh_ptr->veh_turn_dly = TURNDLY_90DEG;  // used in veh_movmnt_fsm.c
         return (2);      
      }
   }
      
/*
// Original code
   if (*cp == ',')
      {
      *cp++;                                   // next character in string
      turn_angle = string_to_dec_int (cp);     // turn angle converted to integer
      ang_result = 0.288 * (float)turn_angle;  // delay = x/90 * turn_angle
      veh_ptr->veh_turn_dly = (int)ang_result; // used on veh_movmnt_fsm.c
      return (1);
      }
*/
   else {
      veh_ptr->veh_turn_dly = TURNDLY_90DEG;  // used in veh_movmnt_fsm.c
      return (2);
   }
}
   
// ********************************************
// Vehicle right turn; Manual mode
int veh_rTrn_cmd()
{
	veh_ptr->manual_cmd_mode = 1;  // manual mode; No auto obstacle detection mode using HC-SR04
	return (1);
}

// ********************************************
// Vehicle left turn; Manual mode
int veh_lTrn_cmd()
{
	veh_ptr->manual_cmd_mode = 2;  // manual mode; No auto obstacle detection mode using HC-SR04
	return (1);
}

// ********************************************
// Vehicle backward movement; Manual mode
int veh_bwd_cmd()
{
	veh_ptr->manual_cmd_mode = 3;  // manual mode; No auto obstacle detection mode using HC-SR04
	return (1);
}

// ********************************************
// Routine to configure PWM duty cycle to control vehicle speed.
int veh_speed_cmd(char *cp_input) {
	   unsigned char spd_dutyCycle_char = '0';
	   //
	   if (*cp_input != ',') {
		   // set to default speed if no duty cycle value included
		   // dutyCycle=(1 to 20); multiplier=250
	       motor_set_dutyCycle(veh_ptr->dutyCycle_primary);      // set motor speeds; defined in motor.c and set in main.c
	   }
	   else {
	       *cp_input++;                                            // inc to next parameter
	       spd_dutyCycle_char = *cp_input;
	       veh_ptr->dutyCycle_primary = char_to_hex_int (spd_dutyCycle_char);   // convert speed duty cycle to integer
	       // Check duty cycle limits.  Must be between 2 and 20, inclusive
	       if (veh_ptr->dutyCycle_primary>1 && veh_ptr->dutyCycle_primary<DCYCLE_MAX) {
	    	   motor_set_dutyCycle(veh_ptr->dutyCycle_primary);    // set motor speeds
	       }
	       else {
			   // set to default speed if no duty cycle value included
	    	   motor_set_dutyCycle(DCYCLE_PRIMARY);    // set motor speeds
	       }
	   }
   return (1);
   }

         
// ****************************
// MEMORY TASKS
// ****************************
// Task to dump 9F bytes of memory starting at the address specified.
int memory_addr (unsigned int st_address)
   {                                // function memory_addr
//   printf("Inside memory_addr().\n");
   uart_puts(UART_ID, "Inside memory_addr().\r\n");
   return(0);     // terminiate function, need to unstack
   }

// Task to zero microcontroller SRAM
int memory_zero (unsigned int st_address)
   {                                // function memory_zero
//   printf("Inside memory_zero().\n");
   uart_puts(UART_ID, "Inside memory_zero().\r\n");
   return(0);     // terminiate function, need to unstack
   }

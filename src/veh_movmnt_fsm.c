// Aug 11, 2024
// FSMs for vPioRxDeferredIntrHandlerTask: HC_SR04 pulse measurement task

// Two state machines, one for forward movement and the other for backward
// movement.  These state machines are called depending on the distance to
// an obstacle.  If the obstacle is less than 10 cm, then the backward
// FSM is called, otherwise the forward FSM is called.  One of these two FSMs
// is called once at the end of a 60 mSec period when the HC-SR04 pulse is
// measured to determine the distance to an obstacle. A PIO state machine is
// used for the 60 mSec delay between ultrasonic pulse measurements.  An 
// Iterrupt Service Routine (ISR) is called when the PIO SM writes the pulse
// measurement to the PIO TX FIFO.

// Each state executes, then waits for next ultrasonic measurement which
// is 60 mSec so there is a 60mSec delay in state transitions.  The delay
// is enough time for the motors to transition to a new movement.

#include "pico/stdlib.h"

#include "motor.h"
#include "cmd_monitor.h"
#include "defs.h"

// Local Variables
uint32_t veh_state_rt = VEHFORWARD_MAN1;  // Right turn state machine
uint32_t veh_state_lt = VEHFORWARD_MAN2;  // Left turn state machine
uint32_t veh_state_bw = VEHFORWARD_MAN3;  // Backward state machine
uint32_t turn_cnt_dly = 0;          // count the number of 60mSec measurements
uint32_t backward_cnt_dly = 0;      // Backward count for number of 60mSec periods; Manual mode
//uint32_t veh_turn_dly;              // vechicle turn delay in 60 mSec increments

//-----------------------------------------
// Move forward until obstacle is detected.
//-----------------------------------------
// Forward Vehicle Movement
// Note: The motors are configured forward and PWM pins enabled in method veh_fwd_cmd()
// in the module monitor.c when the forward command is received from the user
// via the Cypress Bluetooth module.
void forward_auto_fsm(void)
{
	// Start of Forward Movement FSM
	switch (fsm_ptr->veh_state) {

	// Vehicle is stopped and ready to go forward
	case VEHSTOP_GOFOR_AUTO:
		motor_forward();                // set to CW
//		Motor_pin_enable();             // motor driver pins
		fsm_ptr->veh_state = VEHFORWARD_AUTO;    // next state: Forward
		break;

	// Continue forward until obstacle is detected
	case VEHFORWARD_AUTO:
		// Check for obstacle
		if (fsm_ptr->cm <= MINDIS_STOPMTRS) {
			fsm_ptr->veh_state = VEHSTOP_INITBWD_AUTO;  // initiate backward movement
		}
		else {
			fsm_ptr->veh_state = VEHFORWARD_AUTO;    // stay in same state until obstacle detected or User command
		}
		break;

	// ********************
	// Obstacle detected. Move backward.
	// ********************
	// Vehicle was moving forward but detected an obstacle less than x cm away.
	case VEHSTOP_INITBWD_AUTO:
//		Motor_pin_disable();               // disable outputs to motors but continue PWM
		veh_stop_cmd();                    // stop motors
//		GPIO_PORTF_DATA_R &= ~0x0E;        // turn off all three LEDs
//		GPIO_PORTF_DATA_R |= 0x02;         // turn on Red LED
		fsm_ptr->veh_state = VEHSTOP_GOBACK_AUTO;        // next state: Stop, then go backward
		break;

	// Prepare to move vechicle backward
	case VEHSTOP_GOBACK_AUTO:
		motor_backward();                 // set to CCW
//		Motor_pin_enable();               // motor driver pins
		fsm_ptr->veh_state = VEHBACKWARD_AUTO;          // next state: Backward
		break;

	// Obstacle less than x cm away, continue moving backward
	case VEHBACKWARD_AUTO:
		// Check for obstacle
		if (fsm_ptr->cm >= MINDIS_STOPMTRS) {
			// Vechicle was moving back away from obstacle but now it is more than x cm
			// away so stop and initiate a turn.
//			Motor_pin_disable();            // disable outputs to motors but continue PWM
			veh_stop_cmd();               // stop motors
			fsm_ptr->veh_state = VEHSTOP_INITRIGHT_AUTO;  // next state: initiate right turn
		}
		else {
			fsm_ptr->veh_state = VEHBACKWARD_AUTO;      // stay in same state, contine backward
		}
		break;

	// *******************
	// Right Turn
	// *******************
	// Initiate a turn after moving back from obstacle
	case VEHSTOP_INITRIGHT_AUTO:
		// set to default speed for consistent speed during turns
 	    motor_set_dutyCycle(veh_ptr->dutyCycle_turn);   // set motor speeds
		motor_turnRight();              // set motors for right turn
//		Motor_pin_enable();             // motor driver pins
		turn_cnt_dly = 0;               // clr delay counter
		fsm_ptr->veh_state = VEHSTOP_TURNRIGHT_AUTO;  // next state: process right turn
		break;

	// Wait until the turn is complete then stop motors and go to prepare forward state
	case VEHSTOP_TURNRIGHT_AUTO:
	      if (turn_cnt_dly > TURNDLY_90DEG) {  // delay until turn is complete
//		  Motor_pin_disable();                 // disable outputs to motors but continue PWM
		  veh_stop_cmd();               // stop motors
		  // restore parameters in use when turn command was issued
		  motor_set_dutyCycle(veh_ptr->dutyCycle_primary);       // set motor speeds
		  fsm_ptr->veh_state = VEHSTOP_GOFOR_AUTO;      // next state: Stop, then go forward
	  }
	  else {
		  // continue turning
	      turn_cnt_dly += 1;                   // inc counter used to keep track of time during turn
		  fsm_ptr->veh_state = VEHSTOP_TURNRIGHT_AUTO;  // stay in same state
	  }
	  break;


	// ****************
	// Default
	// ****************
	default:
		// do nothing
		fsm_ptr->veh_state = VEHSTOP_GOFOR_AUTO;         // go to start of FSM
		break;
	}
}

//-----------------------------------
// Manual mode. Execute user command.
//-----------------------------------

// ************************
// Execute user command#1
// ************************
// Right hand turn. Manual command = 1
void userCmd_rTrn_fsm(void)
{
	// Start of Right Turn FSM; Manual Mode
	switch (veh_state_rt) {

	// Vehicle was moving forward but received user command to Turn Right.
	case VEHFORWARD_MAN1:
//		Motor_pin_disable();               // disable outputs to motors but continue PWM
		veh_stop_cmd();               // stop motors
		// set to default speed for consistent speed during turns
 	    motor_set_dutyCycle(veh_ptr->dutyCycle_turn);   // set motor speeds
//		GPIO_PORTF_DATA_R &= ~0x0E;        // turn off all three LEDs
//		GPIO_PORTF_DATA_R |= 0x02;         // turn on Red LED
		veh_state_rt = VEHSTOP_INITRIGHT_MAN1;     // next state: initiate right turn
		break;

	// Initiate a turn after moving back from obstacle
	case VEHSTOP_INITRIGHT_MAN1:
		motor_turnRight();              // set motors for right turn
//		Motor_pin_enable();             // motor driver pins
		turn_cnt_dly = 0;               // clr delay counter
		veh_state_rt = VEHSTOP_TURNRIGHT_MAN1;  // next state: process right turn
		break;

	// Wait until the turn is complete then stop motors and go to prepare forward state
	case VEHSTOP_TURNRIGHT_MAN1:
	  if (turn_cnt_dly > veh_ptr->veh_turn_dly) {  // delay until turn is complete
//		  Motor_pin_disable();           // disable outputs to motors but continue PWM
		  veh_stop_cmd();               // stop motors
		  // restore parameters in use when turn command was issued
		  motor_set_dutyCycle(veh_ptr->dutyCycle_primary);       // set motor speeds
		  veh_ptr->manual_cmd_mode = 0;           // set to auto obstacle detection mode using HC-SR04
		  veh_state_rt = VEHFORWARD_MAN1;     // next state: restart
	  }
	  else {
		  // continue turning
		  turn_cnt_dly += 1;                   // inc counter used to keep track of time during turn
		  veh_state_rt = VEHSTOP_TURNRIGHT_MAN1;  // stay in same state
	  }
	  break;

	default:
		// do nothing
		veh_state_rt = VEHFORWARD_MAN1;        // restart
		break;
	}
}

// ************************
// Execute user command#2
// ************************
// Left hand turn. Manual command = 2
void userCmd_lTrn_fsm(void)
{
	// Start of Left Turn FSM; Manual Mode
	switch (veh_state_lt) {

	// Vehicle was moving forward but received user command to Turn Left.
	case VEHFORWARD_MAN2:
//		Motor_pin_disable();               // disable outputs to motors but continue PWM
		veh_stop_cmd();               // stop motors
		// set to default speed for consistent speed during turns
 	    motor_set_dutyCycle(veh_ptr->dutyCycle_turn);   // set motor speeds
//		GPIO_PORTF_DATA_R &= ~0x0E;        // turn off all three LEDs
//		GPIO_PORTF_DATA_R |= 0x02;         // turn on Red LED
		veh_state_lt = VEHSTOP_INITLEFT_MAN2;     // next state: initiate left turn
		break;

	// Initiate Left Turn
	case VEHSTOP_INITLEFT_MAN2:
		motor_turnLeft();               // set motors for Left turn
//		Motor_pin_enable();             // motor driver pins
		turn_cnt_dly = 0;               // clr delay counter
		veh_state_lt = VEHSTOP_TURNLEFT_MAN2;   // next state: process left turn
		break;

	// Wait until the turn is complete then stop motors and go to prepare forward state
	case VEHSTOP_TURNLEFT_MAN2:
	  if (turn_cnt_dly > veh_ptr->veh_turn_dly) {  // delay until turn is complete
//		  Motor_pin_disable();           // disable outputs to motors but continue PWM
		  veh_stop_cmd();               // stop motors
		  // restore parameters in use when turn command was issued
		  motor_set_dutyCycle(veh_ptr->dutyCycle_primary);       // set motor speeds
		  veh_ptr->manual_cmd_mode = 0;           // set to auto obstacle detection mode using HC-SR04
		  veh_state_lt = VEHFORWARD_MAN2;     // next state: Stop, then go forward
	  }
	  else {
		  // continue turning
		  turn_cnt_dly += 1;                   // inc counter used to keep track of time during turn
		  veh_state_lt = VEHSTOP_TURNLEFT_MAN2;  // stay in same state
	  }
	  break;

	default:
		// do nothing
		veh_state_lt = VEHFORWARD_MAN2;        // restart
		break;
	}
}

// ************************
// Execute user command#3
// ************************
// Backward Movement. Manual command = 3
void userCmd_bwd_fsm(void)
{
	// Start of Backward Movement FSM; Manual Mode
	switch (veh_state_bw) {

	// Vehicle was moving forward but received user command to go backward.
	case VEHFORWARD_MAN3:
//		Motor_pin_disable();               // disable outputs to motors but continue PWM
		veh_stop_cmd();               // stop motors
//		GPIO_PORTF_DATA_R &= ~0x0E;        // turn off all three LEDs
//		GPIO_PORTF_DATA_R |= 0x02;         // turn on Red LED
		veh_state_bw = VEHSTOP_GOBACK_MAN3;        // next state: Stop, then go backward
		break;

	// Prepare to move vechicle backward
	case VEHSTOP_GOBACK_MAN3:
		motor_backward();                 // set to CCW
//		Motor_pin_enable();               // motor driver pins
		veh_state_bw = VEHBACKWARD_MAN3;          // next state: Backward
		break;

	// Move bachward for about 3 seconds
	case VEHBACKWARD_MAN3:
		// Check 60 mSec counter
		if (backward_cnt_dly > 50) {      // if more than 3 seconds moving backward
			veh_ptr->manual_cmd_mode = 0;          // set to auto obstacle detection mode using HC-SR04
			backward_cnt_dly = 0;         // clear counter
//			Motor_pin_disable();          // disable outputs to motors but continue PWM
			veh_stop_cmd();               // stop motors
			veh_state_bw = VEHFORWARD_MAN3;    // next state: restart
		}
		else {
			backward_cnt_dly += 1;        // inc 60 mSec counter
			veh_state_bw = VEHBACKWARD_MAN3;      // stay in same state, contine backward
		}
		break;

	default:
		// do nothing
		veh_state_bw = VEHFORWARD_MAN3;        // restart
		break;
	}
}

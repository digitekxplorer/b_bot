// Aug 11, 2024 - Original FSM comments
// FSMs for vPioRxDeferredIntrHandlerTask: HC_SR04 pulse measurement task

// One state machine for autonomous obstacle avoidance and manual command
// execution.  The state machine is called depending on the distance to
// an obstacle.  If the obstacle is less than 15 cm, then the reverse
// movement is initiated, otherwise the forward motion is continued.  The FSM
// is called once at the end of a 60 mSec period when the HC-SR04 pulse is
// measured to determine the distance to an obstacle. A PIO state machine is
// used for the 60 mSec delay between ultrasonic pulse measurements.  An
// Iterrupt Service Routine (ISR) is called when the PIO SM writes the pulse
// measurement to the PIO TX FIFO.
//
// Each state executes, then waits for next ultrasonic measurement which
// is 60 mSec so there is a 60mSec delay in state transitions.  The delay
// is enough time for the motors to transition to a new movement.

// April 30, 2025 - Refactored Vehicle Movement FSM Code using Google Gemini
// Single FSM for rover control, handling both autonomous obstacle
// avoidance and manual command execution.
// Called periodically (e.g., every 60ms) after HC-SR04 distance measurement.
// Main function update_veh_state() called from main.c where HC-SR04 distance
// measurements are calculated.

#include "pico/stdlib.h"
#include <stdio.h> // For diagnostic prints in default case

#include "motor.h"
#include "cmd_monitor.h" // Handle command reception
#include "defs.h"        // Contain veh_ptr definition or similar config
#include "rpp_bot/print_num.h"

// --- Configuration Constants ---
// Minimum distance to obstacle before stopping/backing up (in cm)
// distance from HC-SR04 to an obstacle in cm
#define MIN_OBSTACLE_DISTANCE_CM      15    // min distance is 15 cm, otherwise stop motors
// Duration for turns (in 60ms ticks) - Adjust as needed
#define TURN_DURATION_TICKS           10    // Example: 10 * 60ms = 0.6 seconds
//#define TURNDLY_90DEG               10.0  // this value compared to counter in veh_movmnt_fsm.c
// Duration for manual reverse movement (in 60ms ticks) - Adjust as needed
#define MANUAL_REVERSE_DURATION_TICKS 50    // Example: 50 * 60ms = 3.0 seconds

// --- State Definitions ---
typedef enum {
   // Core States
   IDLE,                     // Stopped, awaiting start (e.g., initial state)

   // Autonomous Obstacle Avoidance States
   AUTO_START_FORWARD,       // Preparing to move forward
   AUTO_MOVING_FORWARD,      // Moving forward, checking distance
   AUTO_STOP_INIT_REVERSE,   // Obstacle detected, stopping before reversing
   AUTO_STOP_START_REVERSE,  // Stopped, preparing to reverse
   AUTO_MOVING_REVERSE,      // Reversing away from obstacle
   AUTO_STOP_INIT_TURN,      // Stopped after reversing, preparing to turn (right)
   AUTO_START_TURN,          // Setting up motors for the turn
   AUTO_TURNING,             // Executing the timed turn

   // Manual Control States (Triggered Externally)
   MANUAL_INIT_TURN_RIGHT,   // Command received, preparing right turn
   MANUAL_START_TURN_RIGHT,  // Start right turn
   MANUAL_TURNING_RIGHT,     // Executing timed manual right turn
   MANUAL_INIT_TURN_LEFT,    // Command received, preparing left turn
   MANUAL_START_TURN_LEFT,   // Start left turn
   MANUAL_TURNING_LEFT,      // Executing timed manual left turn
   MANUAL_INIT_REVERSE,      // Command received, preparing reverse move
   MANUAL_START_REVERSE,     // Start reverse move
   MANUAL_MOVING_REVERSE     // Executing timed manual reverse move

} VehicleOperationalState;

// --- State Structure ---
typedef struct {
   VehicleOperationalState current_state;
   uint32_t              state_timer_ticks;    // Counter for timed states (turns, manual reverse)
// uint32_t              obstacle_distance_cm; // Latest distance measurement from HC-SR04

   // Add other dynamic state if needed, e.g., current speed if variable
} VehicleState;

// --- Global State ---
// Encapsulates the rover's current operational status.
// Initial state can be IDLE or AUTO_START_FORWARD depending on desired power-on behavior.
//VehicleState g_rover_state = { .current_state = IDLE, .state_timer_ticks = 0, .obstacle_distance_cm = 999 };
VehicleState g_rover_state = { .current_state = IDLE, .state_timer_ticks = 0 };

// Assume veh_ptr holds configuration like duty cycles
// This needs to be accessible, e.g., declared extern if defined elsewhere, or passed in.
// extern VehicleConfig* veh_ptr; // Example declaration if veh_ptr is global

// --- Internal Helper Functions ---

/**
 * @brief Checks if a timed action (like a turn) has completed. Increments timer.
 *        If completed, stops motors and restores primary speed.
 * @param state Pointer to the vehicle state structure.
 * @param duration_ticks The total number of ticks the action should last.
 * @return true if the action is complete, false otherwise.
 */
static bool check_timed_action_complete(VehicleState *state, uint32_t duration_ticks) {
   // if (turn_cnt_dly > veh_ptr->veh_turn_dly) {  // delay until turn is complete

      // debug
#ifdef FSM_LOG
      uart_puts(UART_ID, "Inside check_timed_action_complete().\r\n");
      uart_puts(UART_ID, "Turn delay = ");
      static char num_str[5];
      sprintf(num_str, "%d", veh_ptr->veh_turn_dly);
      uart_puts(UART_ID, num_str);
      uart_puts(UART_ID, "\r\n");
      uart_puts(UART_ID, "state->state_timer_ticks = ");
//    print_int(state->state_timer_ticks);                  // TODO: not working correctly
      sprintf(num_str, "%d", state->state_timer_ticks);
      uart_puts(UART_ID, num_str);
      uart_puts(UART_ID, "\r\n");
      // end debug
#endif

   if (state->state_timer_ticks > duration_ticks) {
      veh_stop_cmd();
      // Restore primary driving speed
      motor_set_dutyCycle(veh_ptr->dutyCycle_primary);
      return true; // Action complete
   } else {
      state->state_timer_ticks++;
      return false; // Action ongoing
   }
}

// ******************************************
// --- Main State Machine Update Function ---
// ******************************************
/**
 * @brief Updates the rover's state based on sensor input and current state.
 *        This function should be called periodically (e.g., every 60ms).
 *        Requires veh2obs_cm to be updated beforehand.
 *        Manual commands trigger state changes externally by setting g_rover_state.current_state.
 */
void update_veh_state(uint32_t veh2obs_cm) {
   // Note: Obstacle distance (veh2obs_cm) is updated in main.c
   // by the sensor reading task *before* calling this function.

#ifdef FSM_LOG
   uart_puts(UART_ID, "Inside update_veh_state().\r\n");
   static char num_str[5]; // Use different name to avoid scope collision
   sprintf(num_str, "%d", veh2obs_cm) ;
   uart_puts(UART_ID, num_str);
   uart_puts(UART_ID, "\r\n");
#endif

   switch (g_rover_state.current_state) {

      // --- Core States ---
      case IDLE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "IDLE\r\n");
#endif
         // Waiting for a start command (manual or auto)
         // Could transition to AUTO_START_FORWARD automatically or via command.
         // For now, stays IDLE until state is changed externally.
         // Or, could default to starting:
		 // in Auto mode; execute User command
         if ( veh_ptr->veh_fwd_active == true ) {
            if ( veh_ptr->manual_cmd_mode==0 ) {
               g_rover_state.current_state = AUTO_START_FORWARD;
            }
            else {
               uart_puts(UART_ID, "Error in update_veh_state().\r\n");
            }
         }
			
		 // in manual mode; execute User command
         if ( veh_ptr->veh_turn_active == true ) {
            // Right Turn
            if ( veh_ptr->manual_cmd_mode==1 ) {
#ifdef FSM_LOG
               uart_puts(UART_ID, "IDLE Turn Right\r\n");
#endif
               g_rover_state.current_state = MANUAL_INIT_TURN_RIGHT;         // GoTo MANUAL_INIT_TURN_RIGHT
            }
            // Left Trun
            else if ( veh_ptr->manual_cmd_mode==2 ) {  // Vehicle Left turn; Manual mode
               g_rover_state.current_state = MANUAL_INIT_TURN_LEFT;         // GoTo MANUAL_INIT_TURN_LEFT
            }
            else {
               uart_puts(UART_ID, "Error in update_veh_state().\r\n");
            }			
         }
         break;

      // --- Autonomous Operation ---
      case AUTO_START_FORWARD:
         // TODO: Ensure PWM pins enabled here if not already done elsewhere?
         // The original comment mentioned this was done in veh_fwd_cmd in monitor.c
         motor_forward(); // Set motor direction CW
         motor_set_dutyCycle(veh_ptr->dutyCycle_primary); // Ensure correct speed
         g_rover_state.current_state = AUTO_MOVING_FORWARD;
#ifdef FSM_LOG
            uart_puts(UART_ID, "AF1\r\n");
#endif
         break;

      case AUTO_MOVING_FORWARD:
#ifdef FSM_LOG
            uart_puts(UART_ID, "AF2\r\n");
#endif
         // Check for obstacle
//         if (veh2obs_cm <= MIN_OBSTACLE_DISTANCE_CM) {
         if (veh2obs_cm < 15) {
            g_rover_state.current_state = AUTO_STOP_INIT_REVERSE; // Obstacle detected
         }

         // Check for Manual mode commands
         if ( veh_ptr->manual_cmd_mode != 0 ) {
            if (veh_ptr->manual_cmd_mode==1) {  // Vehicle right turn; Manual mode
               g_rover_state.current_state = MANUAL_INIT_TURN_RIGHT;         // GoTo MANUAL_INIT_TURN_RIGHT
            }
            else if (veh_ptr->manual_cmd_mode==2) {  // Vehicle left turn; Manual mode
               g_rover_state.current_state = MANUAL_INIT_TURN_LEFT;         // GoTo MANUAL_INIT_TURN_LEFT
            }
            else {                                  // Vehicle reverse; Manual mode
               g_rover_state.current_state = MANUAL_INIT_REVERSE;         // GoTo MANUAL_INIT_REVERSE
            }

            // reset to AUTO mode
            veh_ptr->manual_cmd_mode = 0;
         }

         // else: Continue forward, stay in this state (no state change needed)
         break;

      // ********************
      // Obstacle detected. Move reverse.
      // ********************
      case AUTO_STOP_INIT_REVERSE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "AR1\r\n");
#endif
         veh_stop_cmd(); // Stop motors
         g_rover_state.current_state = AUTO_STOP_START_REVERSE;
         break;

      case AUTO_STOP_START_REVERSE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "AR2\r\n");
#endif
         motor_backward(); // Set motor direction CCW
         // Ensure correct speed for backing up (might be same as primary or different)
         motor_set_dutyCycle(veh_ptr->dutyCycle_primary);
         g_rover_state.current_state = AUTO_MOVING_REVERSE;
         break;

      case AUTO_MOVING_REVERSE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "AR3\r\n");
#endif
         // Check if clear of obstacle
         if (veh2obs_cm >= MIN_OBSTACLE_DISTANCE_CM) {
            // Now far enough away, stop and initiate turn
            veh_stop_cmd();
            g_rover_state.current_state = AUTO_STOP_INIT_TURN;
         }
         // else: Continue reverse, stay in this state
         break;

      case AUTO_STOP_INIT_TURN:
         veh_stop_cmd(); // Stop motors
         // set to default speed for consistent speed during turns
         motor_set_dutyCycle(veh_ptr->dutyCycle_turn);   // set motor speeds
         // Prepare for the turn (always right in this autonomous example)
         g_rover_state.current_state = AUTO_START_TURN;
         break;

      case AUTO_START_TURN:
         // Start a timed right turn, transition to AUTO_TURNING state
         motor_turnRight();                   // set motors for right turn
         g_rover_state.state_timer_ticks = 0; // Reset turn timer
         // Start a timed right turn, transition to MANUAL_TURNING_RIGHT state
         g_rover_state.current_state = AUTO_TURNING;     // next state: Auto turn right
         break;

      case AUTO_TURNING:
         // Check if the turn duration has elapsed
         // if (turn_cnt_dly > veh_ptr->veh_turn_dly) {  // delay until turn is complete
//       if (check_timed_action_complete(&g_rover_state, TURN_DURATION_TICKS)) {
         if (check_timed_action_complete(&g_rover_state, veh_ptr->veh_turn_dly)) {
            // Turn complete, go back to moving forward
            g_rover_state.current_state = AUTO_START_FORWARD;
         }
         // else: Continue turning, stay in this state (timer incremented in helper)
         break;

      // ****************************************************
      // --- Manual Operation (States are set externally) ---
      // ****************************************************

      // Received user command to Turn Right.
      case MANUAL_INIT_TURN_RIGHT:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MRT1\r\n");
#endif
         veh_stop_cmd();               // stop motors
         // set to default speed for consistent speed during turns
         motor_set_dutyCycle(veh_ptr->dutyCycle_turn);   // set motor speeds
         g_rover_state.current_state = MANUAL_START_TURN_RIGHT;     // next state: start right turn
/*
#ifdef FSM_LOG
         uart_puts(UART_ID, "Inside FSM, RT init state.\r\n");
         static char num_str_rt_init[5]; // Use different name to avoid scope collision
         sprintf(num_str_rt_init, "%d", g_rover_state.current_state) ;
         uart_puts(UART_ID, num_str_rt_init);
         uart_puts(UART_ID, "\r\n");
#endif
*/
         break;

      case MANUAL_START_TURN_RIGHT:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MRT2\r\n");
#endif
         motor_turnRight();                   // set motors for right turn
         g_rover_state.state_timer_ticks = 0; // Reset turn timer
         // Start a timed right turn, transition to MANUAL_TURNING_RIGHT state
         g_rover_state.current_state = MANUAL_TURNING_RIGHT;     // next state: turn right
         break;

      case MANUAL_TURNING_RIGHT:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MRT3\r\n");
#endif
         // Check if the turn duration has elapsed
         if (check_timed_action_complete(&g_rover_state, TURN_DURATION_TICKS)) {
            // Turn complete, revert to autonomous mode (or IDLE if preferred)    TODO: return to IDLE
//            g_rover_state.current_state = AUTO_START_FORWARD;
            // veh_ptr->veh_turn_active == false;                  // not needed with veh_halt_cmd()
            if ( veh_ptr->manual_cmd_mode == 0 ) {
               // Turn complete, revert to autonomous mode (or IDLE if preferred)    TODO: return to IDLE
               g_rover_state.current_state = AUTO_START_FORWARD;
            }
            else {
               veh_ptr->manual_cmd_mode = 0;
               veh_halt_cmd();                          // stop HC-SR04 trigger pulse and 60 mSec ticks
               g_rover_state.current_state = IDLE;
            }
//            veh_ptr->manual_cmd_mode = 0;
//            veh_halt_cmd();                          // stop HC-SR04 trigger pulse and 60 mSec ticks
//            g_rover_state.current_state = IDLE;
            // If you have a specific flag for manual mode, clear it here.
            // veh_ptr->manual_cmd_mode = 0; // Example if using such a flag
         }
         // else: Continue turning
         break;

      // Received user command to Turn Left.
      case MANUAL_INIT_TURN_LEFT:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MLT1\r\n");
#endif
         veh_stop_cmd();               // stop motors
         // set to default speed for consistent speed during turns
         motor_set_dutyCycle(veh_ptr->dutyCycle_turn);   // set motor speeds
         g_rover_state.current_state = MANUAL_START_TURN_LEFT;     // next state: start left turn
         break;

      case MANUAL_START_TURN_LEFT:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MLT2\r\n");
#endif
         motor_turnLeft();                   // set motors for left turn
         g_rover_state.state_timer_ticks = 0; // Reset turn timer
         // Start a timed left turn, transition to MANUAL_TURNING_LEFT state
         g_rover_state.current_state = MANUAL_TURNING_LEFT;     // next state: turn left
         break;

      case MANUAL_TURNING_LEFT:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MLT3\r\n");
#endif
         // Check if the turn duration has elapsed
         if (check_timed_action_complete(&g_rover_state, TURN_DURATION_TICKS)) {
            // Turn complete, revert to autonomous mode                    TODO: return to IDLE
//            g_rover_state.current_state = AUTO_START_FORWARD;
            // veh_ptr->veh_turn_active == false;       // not needed with veh_halt_cmd()

            if ( veh_ptr->manual_cmd_mode == 0 ) {
               // Turn complete, revert to autonomous mode (or IDLE if preferred)    TODO: return to IDLE
               g_rover_state.current_state = AUTO_START_FORWARD;
            }
            else {
               veh_ptr->manual_cmd_mode = 0;
               veh_halt_cmd();                          // stop HC-SR04 trigger pulse and 60 mSec ticks
               g_rover_state.current_state = IDLE;
            }

//            veh_ptr->manual_cmd_mode = 0;
//            veh_halt_cmd();                          // stop HC-SR04 trigger pulse and 60 mSec ticks
//            g_rover_state.current_state = IDLE;
            // veh_ptr->manual_cmd_mode = 0; // Example if using such a flag
         }
         // else: Continue turning
         break;

      // Received user command to move reverse.
      case MANUAL_INIT_REVERSE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MR1\r\n");
#endif
         veh_stop_cmd();               // stop motors
         // Ensure correct speed for manual reverse movement
         motor_set_dutyCycle(veh_ptr->dutyCycle_primary); // Or a specific reverse speed
         g_rover_state.current_state = MANUAL_START_REVERSE;     // next state: start left turn
         break;

      // Need separate state to start motors moving reverse
      case MANUAL_START_REVERSE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MR2\r\n");
#endif
         motor_backward(); // Set motor direction CCW
         g_rover_state.state_timer_ticks = 0; // Reset timer
         g_rover_state.current_state = MANUAL_MOVING_REVERSE;
         break;

      case MANUAL_MOVING_REVERSE:
#ifdef FSM_LOG
         uart_puts(UART_ID, "MR3\r\n");
#endif
         // Check if the manual reverse duration has elapsed
         if (g_rover_state.state_timer_ticks > MANUAL_REVERSE_DURATION_TICKS) {
            veh_stop_cmd(); // Stop after duration
            // Revert to autonomous mode
            g_rover_state.current_state = AUTO_START_FORWARD;
            // veh_ptr->manual_cmd_mode = 0; // Example if using such a flag
         } else {
            g_rover_state.state_timer_ticks++; // Increment timer
         }
         break;


      // --- Default Case ---
      default:
         // Should not happen! Indicates an error or unhandled state.
//       printf("Error: Unhandled rover state: %d! Resetting to IDLE.\n", g_rover_state.current_state);
         veh_stop_cmd(); // Ensure motors are stopped
         g_rover_state.current_state = IDLE; // Reset to a safe state
         break;
   }
}

// --- External Trigger Functions (Examples) ---
// These would likely be called from your command monitor / Bluetooth handler
/*
void trigger_manual_turn_right(void) {
   // Request a manual right turn by setting the state
   g_rover_state.current_state = MANUAL_INIT_TURN_RIGHT;
   // Optional: Set a flag if needed elsewhere
   // veh_ptr->manual_cmd_mode = 1; // Or similar indicator
}

void trigger_manual_turn_left(void) {
   g_rover_state.current_state = MANUAL_INIT_TURN_LEFT;
   // Optional: Set flag
}

void trigger_manual_reverse(void) {
   g_rover_state.current_state = MANUAL_INIT_REVERSE;
   // Optional: Set flag
}
*/

void trigger_vehicle_fsm_reset(void) {
   g_rover_state.current_state = IDLE;
   // Optional: Set flag
}

void trigger_autonomous_mode(void) {
   // Explicitly switch (back) to autonomous mode
   // Could potentially stop first depending on the current manual action
   veh_stop_cmd();   // stop motors only
// veh_halt_cmd();   // stop motors and HC-SR04
   motor_set_dutyCycle(veh_ptr->dutyCycle_primary); // Ensure primary speed
   g_rover_state.current_state = AUTO_START_FORWARD;
   // Optional: Clear manual flag
}

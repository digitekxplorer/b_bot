// August 11, 2024
// May 1, 2025: Refactored Vehicle Movement FSM Code using Google Gemini

//-----------------------------------
// Auto-detection and manual modes.
//-----------------------------------
// Updates the vehicle's state based on sensor input and current state
void update_veh_state(uint32_t veh2obs_cm);


//-----------------------------------
// Manual mode. Execute user command.
//-----------------------------------
// --- External Trigger Functions (Examples) ---
// These are called from command monitor / Bluetooth handler

// Request a manual right turn by setting the state
//void trigger_manual_turn_right(void);

// Request a manual left turn by setting the state
//void trigger_manual_turn_left(void);

// Request a manual reverse movement by setting the state
//void trigger_manual_reverse(void);

// Request vehicle movement FSM reset to IDLE after halt command
void trigger_vehicle_fsm_reset(void);

// Explicitly switch (back) to autonomous mode
void trigger_autonomous_mode(void);


//-----------------------------------
// Internal Helper Functions
//-----------------------------------
/**
 * @brief Initiates a timed turn (stops, sets turn speed, sets motor direction, resets timer).
 * @param state Pointer to the vehicle state structure.
 * @param turn_right True for right turn, false for left turn.
 * @param next_turning_state The state to transition to while turning.
 */
//static void start_timed_turn(VehicleState *state, bool turn_right, VehicleOperationalState next_turning_state);

/**
 * @brief Checks if a timed action (like a turn) has completed. Increments timer.
 *        If completed, stops motors and restores primary speed.
 * @param state Pointer to the vehicle state structure.
 * @param duration_ticks The total number of ticks the action should last.
 * @return true if the action is complete, false otherwise.
 */
//static bool check_timed_action_complete(VehicleState *state, uint32_t duration_ticks);


//-----------------------------------
// Manual mode. Execute user command.
//-----------------------------------

// User command#1
// Right hand turn. Manual command = 1
//void userCmd_rTrn_fsm(void);

// User command#2
// ************************
// Left hand turn. Manual command = 2
//void userCmd_lTrn_fsm(void);

// User command#3
// Backward Movement. Manual command = 3
//void userCmd_bwd_fsm(void);

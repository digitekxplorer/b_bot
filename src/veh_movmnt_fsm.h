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

// Request vehicle movement FSM reset to IDLE after halt command
void trigger_vehicle_fsm_reset(void);

// Explicitly switch (back) to autonomous mode
void trigger_autonomous_mode(void);

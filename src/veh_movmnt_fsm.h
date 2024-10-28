// August 11, 2024

//-----------------------------------
// Auto-detection mode.
//-----------------------------------
// Forward Movement with Auto-detection Finite State Machine
void forward_auto_fsm(void);

//-----------------------------------
// Manual mode. Execute user command.
//-----------------------------------

// User command#1
// Right hand turn. Manual command = 1
void userCmd_rTrn_fsm(void);

// User command#2
// ************************
// Left hand turn. Manual command = 2
void userCmd_lTrn_fsm(void);

// User command#3
// Backward Movement. Manual command = 3
void userCmd_bwd_fsm(void);


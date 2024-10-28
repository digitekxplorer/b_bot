// July 29, 2024
// B-Bot Motor Controls

// complete Motor PWM output setup
void motor_init(void);

// Clear AIN1&2 and BIN1&2 to have motors start in off state
void motors_off(void);


// Enable PWM
void motor_pwm_enable(void);

// Disable PWM
void motor_pwm_disable(void);


// Motors CCW to move forward
void motor_forward(void);

// Motors CW to move backward
void motor_backward(void);

// Motors turn Right
// Motor#1 CCW
// Motor#2 CW
void motor_turnRight(void);

// Motors turn Left
// Motor#1 CW
// Motor#2 CCW
void motor_turnLeft(void);

// Set PWM duty cycle
uint32_t motor_set_dutyCycle(uint32_t dutyCycle);

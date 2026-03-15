#define DEBUG

// Pin Definitions
#define ENCODER    34  // input only
#define LIMIT_CW   32  // OK
#define LIMIT_CCW  33  // OK
#define MOTOR_CW   25  // OK, supporta PWM
#define MOTOR_CCW  26  // OK, supporta PWM

// deadband and minimum PWM
#define MIN_PWM 70
#define DEADBAND_STEPS 2

// PID gain
#define KP 3
#define KI 0
#define KD 0.1
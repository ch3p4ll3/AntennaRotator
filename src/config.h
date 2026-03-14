#define DEBUG

// Pin Definitions
#define ENCODER    34  // input only
#define LIMIT_CW   32  // OK
#define LIMIT_CCW  33  // OK
#define MOTOR_CW   25  // OK, supporta PWM
#define MOTOR_CCW  26  // OK, supporta PWM

// deadband and minimum PWM
#define MIN_PWM 50
#define DEADBAND_STEPS 5

// PID gain
#define KP 2.0
#define KI 0.5
#define KD 0.1
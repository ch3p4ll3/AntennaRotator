// Pin Definitions
#define ENCODER 2          // Opto-encoder output
#define LIMIT_CW 4      // Home position limit switch
#define LIMIT_CCW 5      // Max elevation limit switch
#define MOTOR_CW 7      // Motor CW
#define MOTOR_CCW 9      // Motor CCW

// deadband and minimum PWM
#define MIN_PWM 50
#define DEADBAND_STEPS 5

// PID gain
#define KP 2.0
#define KI 0.5
#define KD 0.1
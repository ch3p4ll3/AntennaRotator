#include <Arduino.h>

// Pin Definitions
#define ENC_A 2          // Opto-encoder output
#define LIMIT_SW1 4      // Home position limit switch
#define LIMIT_SW2 5      // Max elevation limit switch
#define MOTOR_IN1 6      // Motor driver IN1
#define MOTOR_IN2 7      // Motor driver IN2
#define MOTOR_PWM 9      // Motor driver PWM

volatile long encoderCount = 0;  // Encoder pulse count
float pulsesPerDegree = 0;  // Calibration value
int targetAngle = 0;  // Target elevation angle
bool isCalibrated = false;
bool isMoving = false;

// Motion control variables
int motorSpeed = 0; // Current motor speed
const int maxSpeed = 150; // Max motor speed (0-255)
const int rampStep = 5; // Speed step for soft start/stop
const int rampInterval = 30; // Time between speed steps (ms)
unsigned long lastRampTime = 0;

enum MotorState { IDLE, ACCELERATING, MOVING, DECELERATING };
MotorState motorState = IDLE;

void calibrateSensor();
void encoderISR();
void handleSerialInput();
void updateMotor();
void moveToAngle(int angle);


void setup() {
    Serial.begin(9600);

    // Set pin modes
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(LIMIT_SW1, INPUT_PULLUP);
    pinMode(LIMIT_SW2, INPUT_PULLUP);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);

    // Attach interrupt for encoder
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

    // Start calibration
    calibrateSensor();
}

void loop() {
    handleSerialInput(); // Process incoming commands
    updateMotor();       // Handle non-blocking motor control
}

// Interrupt for encoder count
void encoderISR() {
    encoderCount++;
}

// Handle incoming serial commands
void handleSerialInput() {
    if (Serial.available()) {
        int newAngle = Serial.parseInt();
        if (newAngle >= 0 && newAngle <= 130) {
            targetAngle = newAngle;
            moveToAngle(targetAngle);
        }
    }
}

// **Non-Blocking Motor Control**
void updateMotor() {
    unsigned long currentTime = millis();

    if (motorState == ACCELERATING && currentTime - lastRampTime >= rampInterval) {
        lastRampTime = currentTime;
        motorSpeed = min(motorSpeed + rampStep, maxSpeed);
        analogWrite(MOTOR_PWM, motorSpeed);

        if (motorSpeed == maxSpeed) motorState = MOVING;
    }
    
    else if (motorState == MOVING) {
        long targetCount = targetAngle * pulsesPerDegree;
        if (abs(encoderCount - targetCount) <= 5) {
            motorState = DECELERATING;
        }
    }

    else if (motorState == DECELERATING && currentTime - lastRampTime >= rampInterval) {
        lastRampTime = currentTime;
        motorSpeed = max(motorSpeed - rampStep, 0);
        analogWrite(MOTOR_PWM, motorSpeed);

        if (motorSpeed == 0) {
            motorState = IDLE;
            isMoving = false;
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, LOW);
        }
    }
}

// **Start Motion Smoothly**
void moveToAngle(int angle) {
    if (!isCalibrated) {
        Serial.println("Error: System not calibrated!");
        return;
    }

    long targetCount = angle * pulsesPerDegree;
    bool clockwise = (targetCount > encoderCount);

    digitalWrite(MOTOR_IN1, clockwise ? HIGH : LOW);
    digitalWrite(MOTOR_IN2, clockwise ? LOW : HIGH);

    motorState = ACCELERATING;
    motorSpeed = 0;
    isMoving = true;
    Serial.print("Moving to ");
    Serial.print(angle);
    Serial.println(" degrees...");
}

// **Calibration**
void calibrateSensor() {
    Serial.println("Starting Calibration...");

    // Move to home position
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    motorSpeed = maxSpeed / 2;
    analogWrite(MOTOR_PWM, motorSpeed);
    
    while (digitalRead(LIMIT_SW1) == HIGH) { handleSerialInput(); } // Non-blocking check

    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    delay(500);
    encoderCount = 0;

    // Move to max position
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, motorSpeed);

    while (digitalRead(LIMIT_SW2) == HIGH) { handleSerialInput(); } // Non-blocking check

    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    delay(500);

    pulsesPerDegree = encoderCount / 130.0;
    Serial.print("Calibration Complete. Pulses per Degree: ");
    Serial.println(pulsesPerDegree);
    isCalibrated = true;
}

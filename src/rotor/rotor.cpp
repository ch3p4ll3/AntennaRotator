#include "Arduino.h"
#include <PID_v1.h>
#include "rotor.h"
#include "config.h"

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

void IRAM_ATTR Rotor::isrHandler(void *arg)
{
    Rotor *self = static_cast<Rotor *>(arg);

    unsigned long now = micros();
    if ((now - self->lastPulseTime) < 1000) return; // ignore bounces within 1000µs
    self->lastPulseTime = now;

    if (self->direction)
    {
        self->current_steps++;
    }
    else
    {
        self->current_steps--;
    }
}

Rotor::Rotor(int motor_cw, int motor_ccw, int limit_switch_cw, int limit_switch_ccw, int encoder_pin)
{
    this->motor_cw = motor_cw;
    this->motor_ccw = motor_ccw;
    this->limit_switch_cw = limit_switch_cw;
    this->limit_switch_ccw = limit_switch_ccw;
    this->encoder_pin = encoder_pin;
}

void Rotor::begin(double kp=NAN, double ki=NAN, double kd=NAN)
{
    pinMode(this->motor_ccw, OUTPUT);
    pinMode(this->motor_cw, OUTPUT);
    pinMode(this->limit_switch_cw, INPUT_PULLUP);
    pinMode(this->limit_switch_ccw, INPUT_PULLUP);

    pinMode(this->encoder_pin, INPUT);
    attachInterruptArg(digitalPinToInterrupt(this->encoder_pin), Rotor::isrHandler, this, FALLING);

    this->pid.SetMode(AUTOMATIC);
    this->pid.SetOutputLimits(-255, 255);  // For bidirectional control

    if (!isnan(kp) && !isnan(ki) && !isnan(kd))
        this->pid.SetTunings(kp, ki, kd);
}

void Rotor::loop()
{
    if (!this->is_calibrated)
        return;

    bool at_cw  = digitalRead(this->limit_switch_cw)  == HIGH;
    bool at_ccw = digitalRead(this->limit_switch_ccw) == HIGH;

    if ((at_cw && this->target_steps > this->current_steps) ||
        (at_ccw && this->target_steps < this->current_steps))
    {
        controlMotor(0);
        if (at_cw)  this->current_steps = this->max_degrees * this->steps_per_degree;
        if (at_ccw) this->current_steps = 0;

        this->target_steps = this->current_steps;

        // reset PID state to avoid integral windup
        this->pid.SetMode(MANUAL);
        this->pid.SetMode(AUTOMATIC);
        return;
    }

    // Deadband
    if (abs(this->target_steps - this->current_steps) <= DEADBAND_STEPS)
    {
        this->target_steps = this->current_steps;
        controlMotor(0);
        return;
    }

    this->input = (double)this->current_steps;
    this->setpoint = (double)this->target_steps;
    this->direction = this->target_steps > this->current_steps;

    this->pid.Compute();  // Update output


    controlMotor(this->output);
}

void Rotor::calibrate()
{
    DEBUG_PRINTLN("CALIBRATING...");

    this->direction = true;
    controlMotor(255);

    while (digitalRead(this->limit_switch_cw) == LOW)
    {
        delay(1);
    }

    DEBUG_PRINTLN("Rotor to CW stop");

    controlMotor(0);

    this->current_steps = 0;

    this->direction = false;
    controlMotor(-255);

    DEBUG_PRINTLN(digitalRead(this->limit_switch_ccw));

    while (digitalRead(this->limit_switch_ccw) == LOW)
    {
        delay(1);
    }

    DEBUG_PRINTLN("Rotor to CCW stop");

    controlMotor(0);

    DEBUG_PRINTLN(this->current_steps);

    this->steps_per_degree = abs(this->current_steps) / this->max_degrees;
    this->current_steps = 0;

    this->is_calibrated = true;

    DEBUG_PRINT("Pulses/Degree: ");
    DEBUG_PRINTLN(this->steps_per_degree);
}

void Rotor::set_range(float degrees)
{
    this->max_degrees = degrees;
}

void Rotor::set_offset(float degrees)
{
    this->offset = degrees;
}

void Rotor::move_motor(float degrees)
{
    degrees = degrees - this->offset;

    if (degrees > this->max_degrees || degrees < 0)
    {
        DEBUG_PRINTLN("Out of range!");
        return;
    }

    DEBUG_PRINT("Moving to: ");
    DEBUG_PRINTLN(degrees);

    this->target_steps = (int)(degrees * this->steps_per_degree);
    DEBUG_PRINTLN(this->target_steps);
}

void Rotor::move_motor_by_steps(int steps)
{
    int32_t new_target = this->target_steps + steps;
    if (new_target > this->max_degrees * this->steps_per_degree || new_target < 0)
    {
        DEBUG_PRINTLN("Out of range!");
        return;
    }

    this->target_steps = new_target;
}

float Rotor::get_current_position()
{
    if (this->steps_per_degree == 0) return 0.0;
    return (this->current_steps / this->steps_per_degree) + this->offset;
}

void Rotor::controlMotor(int pwmVal){
    int pwm = abs((int)pwmVal);

    if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;

    pwm = constrain(pwm, 0, 255);

    if (pwmVal > 0) {
        analogWrite(this->motor_cw, pwm);
        digitalWrite(this->motor_ccw, LOW);
    } else if (pwmVal < 0) {
        digitalWrite(this->motor_cw, LOW);
        analogWrite(this->motor_ccw, pwm);
    } else {
        digitalWrite(this->motor_cw, LOW);
        digitalWrite(this->motor_ccw, LOW);
    }
}

void Rotor::stop_motor()
{
    this->target_steps = this->current_steps;
    controlMotor(0);
}

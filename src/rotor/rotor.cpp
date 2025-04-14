#include "Arduino.h"
#include <PID_v1.h>
#include "rotor.h"

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

    if (self->direction)
    {
        self->current_steps++;
    }
    else
    {
        self->current_steps--;
    }
}

Rotor::Rotor(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw, int encoder_pin)
{
    this->motor_ccw = motor_pin;
    this->motor_cw = motor_direction_pin;
    this->limit_switch_cw = limit_switch_cw;
    this->limit_switch_ccw = limit_switch_ccw;
    this->encoder_pin = encoder_pin;
}

void Rotor::begin(double kp=NULL, double ki=NULL, double kd=NULL)
{
    pinMode(this->motor_ccw, OUTPUT);
    pinMode(this->motor_cw, OUTPUT);
    pinMode(this->limit_switch_cw, INPUT_PULLUP);
    pinMode(this->limit_switch_ccw, INPUT_PULLUP);

    pinMode(this->encoder_pin, INPUT);
    attachInterruptArg(digitalPinToInterrupt(this->encoder_pin), Rotor::isrHandler, this, FALLING);

    this->pid.SetMode(AUTOMATIC);
    this->pid.SetOutputLimits(-255, 255);  // For bidirectional control

    if (kp && ki && kd)
        this->pid.SetTunings(kp, ki, kd);
}

void Rotor::loop()
{
    if (!this->is_calibrated)
        return;

    if (this->target_steps == this->current_steps ||
        (digitalRead(this->limit_switch_cw) == HIGH &&
         this->target_steps > this->current_steps) ||
        (digitalRead(this->limit_switch_ccw) == HIGH &&
         this->target_steps < this->current_steps))
    {
        controlMotor(0);

        if (digitalRead(this->limit_switch_cw) == HIGH){
            this->current_steps = this->max_degrees * this->steps_per_degree;
        }

        if (digitalRead(this->limit_switch_ccw) == HIGH){
            this->current_steps = 0;
        }

        return;
    }

    this->input = (double)this->current_steps;
    this->setpoint = (double)this->target_steps;

    this->pid.Compute();  // Update output

    this->direction = this->target_steps > this->current_steps;

    controlMotor(this->output);
}

void Rotor::calibrate()
{
    DEBUG_PRINTLN("CALIBRATING...");

    controlMotor(255);

    while (digitalRead(this->limit_switch_cw) == LOW)
    {
        delay(10);
    }

    DEBUG_PRINTLN("Rotor to CW stop");

    controlMotor(0);

    this->current_steps = 0;

    controlMotor(-255);

    DEBUG_PRINTLN(digitalRead(this->limit_switch_ccw));

    while (digitalRead(this->limit_switch_ccw) == LOW)
    {
        delay(10);
    }

    DEBUG_PRINTLN("Rotor to CCW stop");

    controlMotor(0);

    DEBUG_PRINTLN(this->current_steps);

    this->steps_per_degree = abs(this->current_steps) / this->max_degrees;
    this->current_steps = 0;
    this->current_degrees = 0;

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
    if (steps > this->max_degrees * this->steps_per_degree || steps < 0)
    {
        DEBUG_PRINTLN("Out of range!");
        return;
    }

    this->target_steps = steps + this->target_steps;
}

float Rotor::get_current_position()
{
    return (this->current_steps / this->steps_per_degree) + this->offset;
}

void Rotor::controlMotor(int pwmVal){
    int pwm = abs((int)pwmVal);
    pwm = constrain(pwm, 0, 255);

    if (pwmVal > 0) {
        analogWrite(this->motor_cw, pwm);
        digitalWrite(this->motor_ccw, LOW);
    } else {
        digitalWrite(this->motor_cw, LOW);
        analogWrite(this->motor_ccw, pwm);
    }
}
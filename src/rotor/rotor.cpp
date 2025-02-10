#include "Arduino.h"
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

Rotor::Rotor(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw, int encoder_pin)
{
    this->motor_ccw = motor_pin;
    this->motor_cw = motor_direction_pin;
    this->limit_switch_cw = limit_switch_cw;
    this->limit_switch_ccw = limit_switch_ccw;
    this->encoder_pin = encoder_pin;
}

#pragma region public methods

void Rotor::begin()
{
    pinMode(this->motor_ccw, OUTPUT);
    pinMode(this->motor_cw, OUTPUT);
    pinMode(this->limit_switch_cw, INPUT_PULLUP);
    pinMode(this->limit_switch_ccw, INPUT_PULLUP);

    pinMode(this->encoder_pin, INPUT);
    attachInterruptArg(digitalPinToInterrupt(this->encoder_pin), Rotor::isrHandler, this, FALLING);
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
        this->stop_motor();

        if (digitalRead(this->limit_switch_cw) == HIGH){
            this->current_steps = this->max_degrees * this->steps_per_degree;
        }

        if (digitalRead(this->limit_switch_ccw) == HIGH){
            this->current_steps = 0;
        }

        return;
    }

    this->direction = this->target_steps > this->current_steps;

    float degrees_left = abs(this->current_steps - this->target_steps) / this->steps_per_degree;
    int speed = 255;

    if (degrees_left <= 30){
        speed = 255 * 0.75;
    }
    else if (degrees_left <= 20){
        speed = 255 * 0.60;
    }

    else if (degrees_left <= 10){
        speed = 255 * 0.50;
    }
    
    DEBUG_PRINTLN(degrees_left);

    this->rotate_motor(static_cast<bool>(this->direction), speed);
}

void Rotor::calibrate()
{
    DEBUG_PRINTLN("CALIBRATING...");

    this->rotate_motor(true);

    while (digitalRead(this->limit_switch_cw) == LOW)
    {
        delay(10);
    }

    DEBUG_PRINTLN("Rotor to CW stop");

    this->stop_motor();

    delay(250);  // wait for motor to completely stop

    this->current_steps = 0;

    this->rotate_motor(false);

    while (digitalRead(this->limit_switch_ccw) == LOW)
    {
        delay(10);
    }

    DEBUG_PRINTLN("Rotor to CCW stop");

    this->stop_motor();

    delay(250);  // wait for motor to completely stop

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
    if (steps + this->target_steps > this->max_degrees * this->steps_per_degree || steps + this->target_steps < 0)
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

#pragma endregion

#pragma region private methods

void Rotor::stop_motor(){
    analogWrite(this->motor_cw, 0);
    analogWrite(this->motor_ccw, 0);

    //this->target_steps = this->current_steps;
}

void Rotor::rotate_motor(bool direction, int speed){
    if (direction){
        analogWrite(this->motor_cw, speed);
        analogWrite(this->motor_ccw, 0);
    }

    else {
        analogWrite(this->motor_cw, 0);
        analogWrite(this->motor_ccw, speed);
    }
}

#pragma endregion
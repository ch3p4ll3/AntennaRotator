#include <Arduino.h>
#include <ArduinoLog.h>

#include <PID_v1.h>
#include "rotor.h"
#include "config.h"


#ifdef DEBUG
    #define DEBUG_PRINT(x) Log.noticeln(x)
    #define DEBUG_PRINTLN(x)      Log.noticeln(x)
    #define DEBUG_PRINTF(x, ...)  Log.noticeln(x, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(x, ...)
#endif


void IRAM_ATTR Rotor::isrHandler(void *arg)
{
    Rotor *self = static_cast<Rotor *>(arg);

    // using 74HC14. Don't need to worry about bouncing
    // if (digitalRead(self->encoder_pin) != LOW) return;

    // uint32_t now = micros();
    // uint32_t dt = now - self->lastPulseTime;

    // uint32_t debounce = self->lastPeriod / 3;   // dynamic debounce

    // if (dt < debounce)
    //     return;

    // self->lastPeriod = dt;
    // self->lastPulseTime = now;


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
    attachInterruptArg(digitalPinToInterrupt(this->encoder_pin), Rotor::isrHandler, this, RISING);

    this->pid.SetMode(AUTOMATIC);
    this->pid.SetOutputLimits(-255, 255);  // For bidirectional control
    this->pid.SetSampleTime(20);

    if (!isnan(kp) && !isnan(ki) && !isnan(kd))
        this->pid.SetTunings(kp, ki, kd);

    SettingsData data;
    this->settings->getSettings(&data);

    this->offset = data.offset;
    this->max_degrees = data.range;
    this->steps_per_degree = data.stepsPerDegree;

    DEBUG_PRINTF("offset %F, maxdeg: %F, steps %F", offset,max_degrees, steps_per_degree);
}

void Rotor::loop()
{
    if (!this->is_calibrated)
        return;

    bool at_cw  = digitalRead(this->limit_switch_cw)  == HIGH;
    bool at_ccw = digitalRead(this->limit_switch_ccw) == HIGH;

    if (at_cw)
        this->current_steps = (long)(this->max_degrees * this->steps_per_degree);

    if (at_ccw)
        this->current_steps = 0;
    
        // block movement INTO a limit
    if ((at_cw && this->target_steps >= this->current_steps) ||
        (at_ccw && this->target_steps <= this->current_steps))
    {
        pid.SetMode(MANUAL);
        controlMotor(0);

        this->target_steps = this->current_steps;
        this->output = 0;
        pid.SetMode(AUTOMATIC);

        return;
    }

    // Deadband
    // if (abs(this->target_steps - this->current_steps) <= DEADBAND_STEPS)
    // {
    //     controlMotor(0);
    //     return;
    // }

    this->input = (double)this->current_steps;
    this->setpoint = (double)this->target_steps;

    if (this->pid.Compute()){
        // DEBUG_PRINTF("input: %F, setpoint: %F, out: %F, at_cw: %d, at_ccw: %d", this->input, this->setpoint, this->output, at_cw, at_ccw);
        controlMotor(this->output);
    }
}

void Rotor::calibrate()
{
    DEBUG_PRINTLN("Calibrating...");

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

    while (digitalRead(this->limit_switch_ccw) == LOW)
    {
        delay(1);
    }

    DEBUG_PRINTLN("Rotor to CCW stop");

    controlMotor(0);

    DEBUG_PRINTF("Current steps: %d", this->current_steps);

    this->steps_per_degree = abs(this->current_steps) / this->max_degrees;
    this->current_steps = 0;

    this->is_calibrated = true;

    DEBUG_PRINTF("Calibration complete. Steps per degree: %F", this->steps_per_degree);

    SettingsData data;
    this->settings->getSettings(&data);

    data.stepsPerDegree = this->steps_per_degree;
    this->settings->setSettings(&data);
}

void Rotor::set_range(float degrees)
{
    this->max_degrees = degrees;

    SettingsData data;
    this->settings->getSettings(&data);

    data.range = degrees;
    this->settings->setSettings(&data);
}

void Rotor::set_offset(float degrees)
{
    this->offset = degrees;
    
    SettingsData data;
    this->settings->getSettings(&data);

    data.offset = degrees;
    this->settings->setSettings(&data);
}

void Rotor::move_motor(float degrees)
{
    if (!this->is_calibrated)
        return;

    degrees = degrees - this->offset;

    if (degrees > this->max_degrees || degrees < 0)
    {
        DEBUG_PRINTLN("Out of range!");
        return;
    }

    DEBUG_PRINTF("Moving to: %F degrees", degrees);

    this->target_steps = (int)(degrees * this->steps_per_degree);
    DEBUG_PRINTF("Target steps: %d", this->target_steps);
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
    if (this->steps_per_degree == 0 || !this->is_calibrated) return 0.0 + this->offset; // avoid division by zero
    return (this->current_steps / this->steps_per_degree) + this->offset;
}

float Rotor::get_range()
{
    return this->max_degrees;
}

float Rotor::get_offset()
{
    return this->offset;
}

void Rotor::controlMotor(int pwmVal){
    int pwm = abs((int)pwmVal);

    if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;

    pwm = constrain(pwm, 0, 255);

    if (pwmVal > 0) {
        this->direction = true;
        analogWrite(this->motor_cw, pwm);
        analogWrite(this->motor_ccw, 0);
    } else if (pwmVal < 0) {
        this->direction = false;
        analogWrite(this->motor_cw, 0);
        analogWrite(this->motor_ccw, pwm);
    } else {
        analogWrite(this->motor_cw, 0);
        analogWrite(this->motor_ccw, 0);
    }
}

void Rotor::stop_motor()
{
    controlMotor(0);
    delay(20);
    this->target_steps = this->current_steps;
}

void Rotor::set_settings(Settings *settings) {
    this->settings = settings;
}
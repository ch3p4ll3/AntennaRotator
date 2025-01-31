#include "Arduino.h"
#include "rotator.h"

#ifdef DEBUG
  #define DEBUG_PRINT(x) DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif


void IRAM_ATTR Rotator::encoderISR(){
    if (instance){
        if (instance->direction)
            instance->current_steps++;
        else
            instance->current_steps--;
    }
}


Rotator::Rotator(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw, int encoder_pin){
    this->motor_pin = motor_pin;
    this->motor_direction_pin = motor_direction_pin;
    this->limit_switch_cw = limit_switch_cw;
    this->limit_switch_ccw = limit_switch_ccw;
    this->encoder_pin = encoder_pin;

    instance = this;
}


void Rotator::begin(){
    pinMode(this->motor_pin, OUTPUT);
    pinMode(this->motor_direction_pin, OUTPUT);
    pinMode(this->encoder_pin, INPUT);
    pinMode(this->limit_switch_cw, INPUT_PULLUP);
    pinMode(this->limit_switch_ccw, INPUT_PULLUP);
    
    attachInterrupt(this->encoder_pin, encoderISR, RISING);
}

void Rotator::loop(){
    if (this->target_steps == this->current_steps || digitalRead(this->limit_switch_cw) == LOW || digitalRead(this->limit_switch_ccw) == LOW){
        analogWrite(this->motor_pin, 0);
        return;
    }

    this->direction = this->target_steps > this->current_steps;
    int motor_speed = 255;
    float status = this->current_steps / this->target_steps;

    if (status > 0.8){
        motor_speed = motor_speed / 1.5;
    }
    else if (status > 0.9){
        motor_speed = motor_speed / 2;
    }
    else if (status > 0.95){
        motor_speed = motor_speed / 4;
    }

    digitalWrite(this->direction, this->direction);
    analogWrite(this->motor_pin, motor_speed);
}

void Rotator::calibrate(){
    DEBUG_PRINTLN("CALIBRATING...");

    digitalWrite(this->motor_direction_pin, HIGH);
    analogWrite(this->motor_pin, 255);

    while (digitalRead(this->limit_switch_cw) == HIGH) { }

    analogWrite(this->motor_pin, 0);
    this->current_steps = 0;

    digitalWrite(this->motor_direction_pin, LOW);
    analogWrite(this->motor_pin, 255);

    while (digitalRead(this->limit_switch_ccw) == HIGH) { }

    analogWrite(this->motor_pin, 0);
    this->pulses_per_degree = this->current_steps / this->max_degrees;
    DEBUG_PRINT("Pulses/Degree: ");
    DEBUG_PRINTLN(this->current_steps);
}


void Rotator::set_range(float degrees){
    this->max_degrees = degrees;
}


void Rotator::move_motor(float degrees){
    if (degrees > this->max_degrees || degrees < 0){
        DEBUG_PRINTLN("Out of range!");
        return;
    }

    this->target_steps = (int)degrees * this->pulses_per_degree;
}


void Rotator::move_motor(int steps){
    if (steps > this->max_degrees * this->pulses_per_degree || steps < 0){
        DEBUG_PRINTLN("Out of range!");
        return;
    }

    this->target_steps = steps;
}
#include "Arduino.h"

#pragma once


class Rotor
{
private:
    int motor_ccw;
    int motor_cw;
    int limit_switch_cw;
    int limit_switch_ccw;
    int encoder_pin;

    volatile bool direction = true; // true = cw
    float max_degrees = 360;
    float steps_per_degree = 100; // to calibrate

    volatile long target_steps = 0;
    volatile long current_steps = 0;
    volatile float current_degrees = 0;

    volatile unsigned long lastPulseTime = 0;

    float offset = 0;

    bool is_calibrated = false;

    void rotate_motor(bool direction, int speed=255);

    static void IRAM_ATTR isrHandler(void *arg);

public:
    Rotor(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw, int encoder_pin);
    void begin();
    void loop();
    void calibrate();

    void set_range(float degrees);
    void set_offset(float degrees);

    void move_motor(float degrees);
    void move_motor_by_steps(int steps);
    void stop_motor();
    float get_current_position();
};
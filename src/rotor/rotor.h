#include "Arduino.h"

#pragma once


struct StoredData {
    int current_steps;
};


class Rotor
{
private:
    int motor_ccw;
    int motor_cw;
    int limit_switch_cw;
    int limit_switch_ccw;
    int encoder_pin;

    int eeprom_index = 0;

    volatile bool direction = true; // true = cw
    float max_degrees = 360;
    float pulses_per_degree = 4; // to calibrate

    volatile int target_steps = 0;
    volatile int current_steps = 0;
    volatile float current_degrees = 0;

    float offset = 0;

    bool is_calibrated = false;

    void store_data();

public:
    Rotor(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw);
    void begin();
    void loop();
    void calibrate();

    void set_range(float degrees);
    void set_offset(float degrees);

    void move_motor(float degrees);
    void move_motor(int steps);
    float get_current_position();

    void IRAM_ATTR encoderISR();
};
#pragma once
#include "Arduino.h"
#include <PID_v1.h>

#include "../settings/settings.h"


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

    int target_steps = 0;
    volatile long current_steps = 0;

    volatile unsigned long lastPulseTime = 0;
    volatile uint32_t lastPeriod = 10000; // start large

    float offset = 0;

    bool is_calibrated = false;

    static void IRAM_ATTR isrHandler(void *arg);

    double input = 0;
    double output = 0;
    double setpoint = 0;
    PID pid = PID(&input, &output, &setpoint, 2.0, 0.5, 0.1, DIRECT);
    void controlMotor(int pwmVal);

    Settings settings;

public:
    Rotor(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw, int encoder_pin);
    void begin(double kp, double ki, double kd);
    void loop();
    void calibrate();

    void set_range(float degrees);
    void set_offset(float degrees);

    void move_motor(float degrees);
    void move_motor_by_steps(int steps);
    float get_current_position();
    float get_range();
    float get_offset();
    void stop_motor();

    void set_settings(const Settings& settings);
};
class Rotator{
    private:
        int motor_pin;
        int motor_direction_pin;
        int limit_switch_cw;
        int limit_switch_ccw;
        int encoder_pin;

        bool direction = true;  // true = cw
        float max_degrees = 360;
        int pulses_per_degree = 5;  // to calibrate

        int target_steps = 0;
        volatile int current_steps = 0;
        float current_degrees = 0;

        static void IRAM_ATTR encoderISR();

    public:
        static Rotator* instance;  // Declare the static instance pointer
        Rotator(int motor_pin, int motor_direction_pin, int limit_switch_cw, int limit_switch_ccw, int encoder_pin);
        void begin();
        void loop();
        void calibrate();

        void set_range(float degrees);
        void move_motor(float degrees);
        void move_motor(int steps);
};
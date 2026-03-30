#include "../rotor/rotor.h"
#include "../settings/settings.h"


struct Position {
    float elevation;
    float azimuth;
};


class Rotator{
    private:
        Rotor* azimuth = nullptr;
        Rotor* elevation = nullptr;
    
    public:
        Rotator(Rotor* azimuth, Rotor* elevation);
        void begin(double az_kp, double az_ki, double az_kd, double el_kp, double el_ki, double el_kd);
        void loop();
        void calibrate();

        void set_offset(float azimuth_degrees, float elevation_degrees);

        void set_range(float azimuth_max_degrees, float elevation_max_degrees);
        void move_motor(float azimuth_degrees, float elevation_degrees);
        void move_motor_by_steps(int azimuth_steps, int elevation_steps);
        void stop_motor();

        Position get_current_position();
        Position get_range();
        Position get_offset();

        void set_settings(Settings *azimuth_settings, Settings *eleveation_settings);
};
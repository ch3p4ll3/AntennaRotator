#include "../rotor/rotor.h"


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
        void begin();
        void loop();
        void calibrate();

        void set_offset(float azimuth_degrees, float elevation_degrees);

        void set_range(float azimuth_max_degrees, float elevation_max_degrees);
        void move_motor(float azimuth_degrees, float elevation_degrees);
        void move_motor(int azimuth_steps, int elevation_steps);

        Position get_current_position();
};
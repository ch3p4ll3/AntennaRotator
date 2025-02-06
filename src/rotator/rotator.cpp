#include "Arduino.h"
#include "rotator.h"

Rotator::Rotator(Rotor *azimuth=nullptr, Rotor *elevation=nullptr)
{
    this->azimuth = azimuth;
    this->elevation = elevation;
}

void Rotator::begin()
{
    if (this->azimuth)
        this->azimuth->begin();

    if (this->elevation)
        this->elevation->begin();
}

void Rotator::calibrate()
{
    if (this->azimuth)
        this->azimuth->calibrate();

    if (this->elevation)
        this->elevation->calibrate();
}

void Rotator::loop()
{
    if (this->azimuth)
        this->azimuth->loop();

    if (this->elevation)
        this->elevation->loop();
}

void Rotator::set_range(float azimuth_max_degrees, float elevation_max_degrees)
{
    if (this->azimuth)
        this->azimuth->set_range(azimuth_max_degrees);

    if (this->elevation)
        this->elevation->set_range(elevation_max_degrees);
}

void Rotator::move_motor(float azimuth_degrees, float elevation_degrees)
{
    if (this->azimuth)
        this->azimuth->move_motor(azimuth_degrees);

    if (this->elevation)
        this->elevation->move_motor(elevation_degrees);
}

void Rotator::move_motor(int azimuth_steps, int elevation_steps)
{
    if (this->azimuth)
        this->azimuth->move_motor(azimuth_steps);

    if (this->elevation)
        this->elevation->move_motor(elevation_steps);
}

void Rotator::set_offset(float azimuth_degrees, float elevation_degrees){
    if (this->azimuth)
        this->azimuth->set_offset(azimuth_degrees);

    if(this->elevation)
        this->elevation->set_offset(elevation_degrees);
}

Position Rotator::get_current_position()
{
    Position p;

    if (this->azimuth)
        p.azimuth = this->azimuth->get_current_position();
    else
        p.azimuth = 0.0;

    if (this->elevation)
        p.elevation = this->elevation->get_current_position();
    else
        p.elevation = 0.0;

    return p;
}

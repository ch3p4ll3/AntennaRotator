#include "config.h"

#include "Arduino.h"
#include "./rotator/rotator.h"


Rotator rotator(MOTOR_PWM, MOTOR_DIR, LIMIT_CW, LIMIT_CCW, ENCODER);


void setup(){
    rotator.begin();
    rotator.set_range(130);

    rotator.calibrate();
}


void loop(){
    rotator.loop();
}


void encoderISR(){

}
#include "config.h"
#include "./rotor/rotor.h"
#include "./rotator/rotator.h"

#include <easycomm-parser-types-ctors.h>
#include <easycomm-parser.h>
#include <easycomm-command-callback-handler.h>

#include "Arduino.h"


#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(x, ...)
#endif


EasycommCommandsCallback cb_handler;

Rotor azimuth(MOTOR_CW, MOTOR_CCW, LIMIT_CW, LIMIT_CCW, ENCODER);
Rotator rotator(&azimuth, nullptr);

TaskHandle_t SerialTaskHandle = NULL;

void SerialTask(void *parameter) {
    for (;;) {
        if (Serial2.available()) {
            String line = Serial2.readStringUntil('\n');
            line.trim();
            DEBUG_PRINTF("RX: '%s'\n", line.c_str());
            easycommHandleCommand(line.c_str(), &cb_handler, EasycommParserStandard2, nullptr);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void onAzimuth(const EasycommData *cmd, void *user_data) {
    float targetAz = cmd->as.setAzimuth.azimuth;
    rotator.move_motor(targetAz, 0);
}

void onGetAzimuth(const EasycommData *cmd, void *user_data) {
    Position p = rotator.get_current_position();

    Serial2.print("AZ");
    Serial2.print(p.azimuth, 1);
    Serial2.print(" EL");
    Serial2.println(p.elevation, 1);
}

void onStop(const EasycommData *cmd, void *user_data) {
    // Call your rotator's stop method
    rotator.stop_motor(); 
    #ifdef DEBUG
    DEBUG_PRINTLN("Stop command received");
    #endif
}

void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600);
    Serial2.setTimeout(50);

    rotator.begin(KP, KI, KD, NAN, NAN, NAN);
    rotator.set_range(130, 0);

    rotator.calibrate();

    easycommCommandsCallback(&cb_handler, EasycommParserStandard2);

    // Override registry with Azimuth, Get Azimuth, and Stop
    cb_handler.registry[EasycommIdSetAzimuth] = onAzimuth;
    cb_handler.registry[EasycommIdGetAzimuth] = onGetAzimuth;
    cb_handler.registry[EasycommIdSingleLine] = onGetAzimuth;
    cb_handler.registry[EasycommIdDoStopAzimuthMove] = onStop; // Standard EasyComm 'ST'

    xTaskCreatePinnedToCore(
        SerialTask,         // Task function
        "SerialTask",       // Task name
        10000,             // Stack size (bytes)
        NULL,              // Parameters
        1,                 // Priority
        &SerialTaskHandle,  // Task handle
        1                  // Core 1
    );
}

void loop()
{
    rotator.loop();
}

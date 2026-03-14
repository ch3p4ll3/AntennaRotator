#include "config.h"
#include "./rotor/rotor.h"
#include "./rotator/rotator.h"

#include <easycomm-parser-types-ctors.h>
#include <easycomm-parser.h>
#include <easycomm-command-callback-handler.h>

#include <Arduino.h>
#include <ArduinoLog.h>

#define DEBUG_SERIAL Serial
#define RSERIAL Serial2


#ifdef DEBUG
    #define DEBUG_PRINT(x)          Log.noticeln(x)
    #define DEBUG_PRINTLN(x)        Log.noticeln(x)
    #define DEBUG_PRINTF(x, ...)    Log.noticeln(x, ##__VA_ARGS__)
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
        if (RSERIAL.available()) {
            String line = RSERIAL.readStringUntil('\n');
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
    char buf[16];
    snprintf(buf, sizeof(buf), "AZ%05.1f\n", p.azimuth);  // AZ000.0
    RSERIAL.print(buf);
    RSERIAL.flush();
}

void onGetElevation(const EasycommData *cmd, void *user_data) {
    Position p = rotator.get_current_position();
    char buf[16];
    snprintf(buf, sizeof(buf), "EL%05.1f\n", p.elevation);  // EL000.0
    RSERIAL.print(buf);
    RSERIAL.flush();
}

void onStop(const EasycommData *cmd, void *user_data) {
    // Call your rotator's stop method
    rotator.stop_motor(); 
    #ifdef DEBUG
    Log.infoln("Stop command received");
    #endif
}

void setup()
{
    DEBUG_SERIAL.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &DEBUG_SERIAL);

    RSERIAL.begin(115200);
    RSERIAL.setTimeout(50);

    rotator.begin(KP, KI, KD, NAN, NAN, NAN);
    rotator.set_range(130, 0);

    rotator.calibrate();

    easycommCommandsCallback(&cb_handler, EasycommParserStandard2);

    // Override registry with Azimuth, Get Azimuth, and Stop
    cb_handler.registry[EasycommIdSetAzimuth] = onAzimuth;
    cb_handler.registry[EasycommIdGetAzimuth]   = onGetAzimuth;
    cb_handler.registry[EasycommIdGetElevation] = onGetElevation;
    cb_handler.registry[EasycommIdSingleLine]   = onGetAzimuth; // fallback
    cb_handler.registry[EasycommIdDoStopAzimuthMove] = onStop; // Standard EasyComm 'ST'

    xTaskCreatePinnedToCore(
        SerialTask,         // Task function
        "SerialTask",       // Task name
        10000,             // Stack size (bytes)
        NULL,              // Parameters
        1,                 // Priority
        &SerialTaskHandle,  // Task handle
        0                  // Core 1
    );
}

void loop()
{
    rotator.loop();
}

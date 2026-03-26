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

// Forward declarations
void parseCustomCommands(String line);
void SerialTask(void *parameter);

// Forward declarations for easycomm callbacks
void onSetAzimuth(const EasycommData *cmd, void *user_data);
void onSetElevation(const EasycommData *cmd, void *user_data);
void onSingleLine(const EasycommData *cmd, void *user_data);
void onGetAzimuth(const EasycommData *cmd, void *user_data);
void onGetElevation(const EasycommData *cmd, void *user_data);
void onStop(const EasycommData *cmd, void *user_data);


void setup()
{
    DEBUG_SERIAL.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &DEBUG_SERIAL);

    RSERIAL.begin(115200);
    RSERIAL.setTimeout(50);

    rotator.begin(KP, KI, KD, NAN, NAN, NAN);
    rotator.set_range(90, 0);

    rotator.calibrate();

    easycommCommandsCallback(&cb_handler, EasycommParserStandard2);

    // Override registry with Azimuth, Get Azimuth, and Stop
    cb_handler.registry[EasycommIdSetAzimuth] = onSetAzimuth;
    cb_handler.registry[EasycommIdGetAzimuth]   = onGetAzimuth;
    cb_handler.registry[EasycommIdSetElevation] = onSetElevation;
    cb_handler.registry[EasycommIdGetElevation] = onGetElevation;
    cb_handler.registry[EasycommIdSingleLine]   = onSingleLine; // fallback
    cb_handler.registry[EasycommIdDoStopAzimuthMove] = onStop; // Standard EasyComm 'ST'

    xTaskCreatePinnedToCore(
        SerialTask,         // Task function
        "SerialTask",       // Task name
        10000,             // Stack size (bytes)
        NULL,              // Parameters
        1,                 // Priority
        &SerialTaskHandle,  // Task handle
        0                  // Core 0
    );
}

void loop()
{
    rotator.loop();
}

void SerialTask(void *parameter) {
    for (;;) {
        if (RSERIAL.available()) {
            String line = RSERIAL.readStringUntil('\n');
            DEBUG_PRINTF("RX: '%s'\n", line.c_str());
            line.trim();

            bool status = easycommHandleCommand(line.c_str(), &cb_handler, EasycommParserStandard2, nullptr);

            if (!status) {
                DEBUG_PRINTLN("Command not recognized by easycomm parser, trying custom parser...");
                parseCustomCommands(line);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void parseCustomCommands(String line) {
    // This function can be used to parse any custom commands that are not handled by the easycomm parser
    // For example, you can implement commands like "RECAL", "REBOOT", "RANGE AZ?", etc. here
    if (line == "RECAL") {
        DEBUG_PRINTLN("Recalibrating...");
        rotator.calibrate();
        RSERIAL.println("RECALIBRATED");

    } else if (line == "REBOOT") {
        DEBUG_PRINTLN("Rebooting...");
        delay(100);
        ESP.restart();

    } else if (line == "RANGE AZ?") {
        float range = rotator.get_range().azimuth;
        RSERIAL.printf("RANGE AZ%05.1f\n", range);

    } else if (line == "RANGE EL?") {
        float range = rotator.get_range().elevation;
        RSERIAL.printf("RANGE EL%05.1f\n", range);

    } else if (line == "OFFSET AZ?") {
        float offset = rotator.get_offset().azimuth;
        RSERIAL.printf("OFFSET AZ%05.1f\n", offset);

    } else if (line == "OFFSET EL?") {
        float offset = rotator.get_offset().elevation;
        RSERIAL.printf("OFFSET EL%05.1f\n", offset);

    } else if (line.startsWith("RANGE AZ ")) {
        float range = line.substring(9).toFloat();
        rotator.set_range(range, rotator.get_range().elevation);
        DEBUG_PRINTF("AZ range set to: %F", range);
        RSERIAL.printf("RANGE AZ%05.1f\n", range);

    } else if (line.startsWith("RANGE EL ")) {
        float range = line.substring(9).toFloat();
        rotator.set_range(rotator.get_range().azimuth, range);
        DEBUG_PRINTF("EL range set to: %F", range);
        RSERIAL.printf("RANGE EL%05.1f\n", range);

    } else if (line.startsWith("OFFSET AZ ")) {
        float offset = line.substring(10).toFloat();
        rotator.set_offset(offset, rotator.get_offset().elevation);
        DEBUG_PRINTF("AZ offset set to: %F", offset);
        RSERIAL.printf("OFFSET AZ%05.1f\n", offset);

    } else if (line.startsWith("OFFSET EL ")) {
        float offset = line.substring(10).toFloat();
        rotator.set_offset(rotator.get_offset().azimuth, offset);
        DEBUG_PRINTF("EL offset set to: %F", offset);
        RSERIAL.printf("OFFSET EL%05.1f\n", offset);
    
    } else if (line.startsWith("AZ") && line.indexOf("EL") > 0){
        int elIdx = line.indexOf("EL");

        String azStr = line.substring(2, elIdx);
        azStr.trim();
        float targetAz = azStr.toFloat();

        String elStr = line.substring(elIdx + 2);
        elStr.trim();
        float targetEl = elStr.toFloat();

        rotator.move_motor(targetAz, targetEl);
    }
    else {
        DEBUG_PRINTLN("Unknown command");
        RSERIAL.println("ERROR: Unknown command");
    }
}

void onSetAzimuth(const EasycommData *cmd, void *user_data) {
    float targetAz = cmd->as.setAzimuth.azimuth;
    rotator.move_motor(targetAz, 0);
}

void onSetElevation(const EasycommData *cmd, void *user_data) {
    float targetEl = cmd->as.setElevation.elevation;
    rotator.move_motor(0, targetEl);
}

void onSingleLine(const EasycommData *cmd, void *user_data){
    float targetAz = cmd->as.singleLine.azimuth;
    float targetEl = cmd->as.singleLine.elevation;

    rotator.move_motor(targetAz, targetEl);
}

void onGetAzimuth(const EasycommData *cmd, void *user_data) {
    Position p = rotator.get_current_position();
    char buf[16];
    snprintf(buf, sizeof(buf), "AZ%05.1f\n", p.azimuth);  // AZ000.0
    RSERIAL.println(buf);
    RSERIAL.flush();
}

void onGetElevation(const EasycommData *cmd, void *user_data) {
    Position p = rotator.get_current_position();
    char buf[16];
    snprintf(buf, sizeof(buf), "EL%05.1f\n", p.elevation);  // EL000.0
    RSERIAL.println(buf);
    RSERIAL.flush();
}


void onStop(const EasycommData *cmd, void *user_data) {
    // Call your rotator's stop method
    rotator.stop_motor(); 
    #ifdef DEBUG
    DEBUG_PRINTLN("Stop command received");
    #endif
}
#include "config.h"
#include "./rotor/rotor.h"
#include "./rotator/rotator.h"

#include <AsyncTCP.h>
#include <DNSServer.h>
#include <WiFi.h>

#include "Arduino.h"

static DNSServer DNS;

Rotor azimuth(MOTOR_PWM, MOTOR_DIR, LIMIT_CW, LIMIT_CCW, ENCODER);
Rotator rotator(&azimuth, nullptr);

// function definition
void connect_to_wifi();
void coreTask(void *pvParameters);

static void handleData(void *arg, AsyncClient *client, void *data, size_t len)
{
    Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());

    String decodedData = String((uint8_t *)data, len);
    String toSendString;
    Position p = rotator.get_current_position();

    if (decodedData.startsWith("p"))
    {
        Serial.println("Get current position");
        toSendString = String(p.azimuth);
        toSendString += "\n";
        toSendString += String(p.elevation);
        toSendString += "\n";
    }

    else if (decodedData.startsWith("P"))
    {
        Serial.println("Set position" + decodedData);
        String numbers = decodedData.substring(2);
        int indexOfSpace = numbers.indexOf(' ');

        String azim = numbers.substring(0, indexOfSpace);
        String elev = numbers.substring(indexOfSpace + 1);

        rotator.move_motor(azim.toFloat(), elev.toFloat());

        toSendString = "RPRT 0\n";
    }

    else if (decodedData.startsWith("S"))
    {
        toSendString = "RPRT 0\n";
    }

    if (client->space() > strlen(toSendString.c_str()) && client->canSend())
    {
        client->add(toSendString.c_str(), strlen(toSendString.c_str()));
        client->send();
    }
}

static void handleError(void *arg, AsyncClient *client, int8_t error)
{
    Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleDisconnect(void *arg, AsyncClient *client)
{
    Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
}

static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time)
{
    Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}

static void handleNewClient(void *arg, AsyncClient *client)
{
    Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());
    // register events
    client->onData(&handleData, NULL);
    client->onError(&handleError, NULL);
    client->onDisconnect(&handleDisconnect, NULL);
    client->onTimeout(&handleTimeOut, NULL);
}

void setup()
{
    xTaskCreatePinnedToCore(
        coreTask,   /* Function to implement the task */
        "coreTask", /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        0,          /* Priority of the task */
        NULL,       /* Task handle. */
        0);         /* Core where the task should run */

    rotator.begin();
    rotator.set_range(130, 0);

    rotator.calibrate();
}

void loop()
{
    rotator.loop();
}

void coreTask(void *pvParameters)
{
    connect_to_wifi();

    AsyncServer *server = new AsyncServer(TCP_SERVER_PORT);
    server->onClient(&handleNewClient, server);
    server->begin();
}

void connect_to_wifi()
{
    WiFi.mode(WIFI_STA); // Optional
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("\nConnecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}
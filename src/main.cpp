#include "config.h"
#include "./rotor/rotor.h"
#include "./rotator/rotator.h"

#include <AsyncTCP.h>
#include <WiFi.h>

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



Rotor azimuth(MOTOR_CW, MOTOR_CCW, LIMIT_CW, LIMIT_CCW, ENCODER);
Rotator rotator(&azimuth, nullptr);

// function definition
void connect_to_wifi();
void init_server();

volatile bool aziumuth_encoder = false;


static void handleData(void *arg, AsyncClient *client, void *data, size_t len)
{
    DEBUG_PRINTF("\n data received from client %s \n", client->remoteIP().toString().c_str());

    String decodedData = String((uint8_t *)data, len);
    String toSendString;

    if (decodedData.startsWith("p"))
    {
        Position p = rotator.get_current_position();
        //DEBUG_PRINTLN("Get current position");
        toSendString = toSendString = String(p.azimuth, 1) + "\n" + String(p.elevation, 1) + "\n";  // 1 decimal point;
    }

    else if (decodedData.startsWith("P"))
    {
        //DEBUG_PRINTLN("Set position" + decodedData);
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
    DEBUG_PRINTF("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleDisconnect(void *arg, AsyncClient *client)
{
    DEBUG_PRINTF("\n client %s disconnected \n", client->remoteIP().toString().c_str());
}

static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time)
{
    DEBUG_PRINTF("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}

static void handleNewClient(void *arg, AsyncClient *client)
{
    DEBUG_PRINTF("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());
    // register events
    client->onData(&handleData, NULL);
    client->onError(&handleError, NULL);
    client->onDisconnect(&handleDisconnect, NULL);
    client->onTimeout(&handleTimeOut, NULL);
}

void setup()
{
    Serial.begin(115200);

    rotator.begin();
    rotator.set_range(130, 0);

    rotator.calibrate();

    init_server();
}

void loop()
{
    rotator.loop();
}

void init_server()
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
    DEBUG_PRINTLN("\nConnecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        DEBUG_PRINT(".");
        delay(100);
    }

    DEBUG_PRINTLN("\nConnected to the WiFi network");
    DEBUG_PRINT("Local ESP32 IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
}
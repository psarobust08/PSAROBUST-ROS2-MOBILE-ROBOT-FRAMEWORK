/*
 * PSA ROBUST - Teensy 4.1
 * Serial Receiver V1.1: velocity + FSM heartbeat
 * ROS 2 -> serial_node -> Teensy 4.1
 * Commands: vx,vy,wz\n OR HB,IDLE|STOPPED|RUNNING\n
 * Baud rate: 115200
 * LED strip and motor control: NOT IMPLEMENTED
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

constexpr uint32_t BAUD_RATE = 115200;
constexpr size_t BUFFER_SIZE = 64;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;

char rxBuffer[BUFFER_SIZE];
size_t rxIndex = 0;

float vx = 0.0f;
float vy = 0.0f;
float wz = 0.0f;

// IDLE also represents the fallback LED state (red breathing).
const char* fsmState = "IDLE";
uint32_t lastHeartbeatMs = 0;
bool heartbeatReceived = false;

void processCommand(const char* data)
{
    if (strncmp(data, "HB,", 3) == 0)
    {
        const char* incomingState = data + 3;
        if (strcmp(incomingState, "IDLE") != 0 &&
            strcmp(incomingState, "STOPPED") != 0 &&
            strcmp(incomingState, "RUNNING") != 0)
        {
            Serial.println("ERR,INVALID_HEARTBEAT");
            return;
        }

        fsmState = strcmp(incomingState, "IDLE") == 0 ? "IDLE" :
                   strcmp(incomingState, "STOPPED") == 0 ? "STOPPED" : "RUNNING";
        lastHeartbeatMs = millis();
        heartbeatReceived = true;
        Serial.print("HB_RX,");
        Serial.println(fsmState);
        return;
    }

    float newVx, newVy, newWz;
    char extra;
    int count = sscanf(data, " %f , %f , %f %c", &newVx, &newVy, &newWz, &extra);

    if (count != 3)
    {
        Serial.println("ERR,INVALID_FORMAT");
        return;
    }

    if (!isfinite(newVx) || !isfinite(newVy) || !isfinite(newWz))
    {
        Serial.println("ERR,INVALID_VALUE");
        return;
    }

    vx = newVx;
    vy = newVy;
    wz = newWz;

    Serial.print("RX | vx=");
    Serial.print(vx, 3);
    Serial.print(" | vy=");
    Serial.print(vy, 3);
    Serial.print(" | wz=");
    Serial.println(wz, 3);
}

void setup()
{
    Serial.begin(BAUD_RATE);
    Serial.println("PSA ROBUST Teensy Serial Receiver V1.1");
    Serial.println("STATUS: READY");
}

void loop()
{
    while (Serial.available() > 0)
    {
        char incoming = static_cast<char>(Serial.read());

        if (incoming == '\n')
        {
            rxBuffer[rxIndex] = '\0';
            if (rxIndex > 0)
            {
                processCommand(rxBuffer);
            }
            rxIndex = 0;
        }
        else if (incoming != '\r')
        {
            if (rxIndex < BUFFER_SIZE - 1)
            {
                rxBuffer[rxIndex++] = incoming;
            }
            else
            {
                rxIndex = 0;
                Serial.println("ERR,BUFFER_OVERFLOW");
            }
        }
    }

    if (heartbeatReceived &&
        static_cast<uint32_t>(millis() - lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS)
    {
        heartbeatReceived = false;
        fsmState = "IDLE";
        Serial.println("HB_TIMEOUT,IDLE");
    }
}

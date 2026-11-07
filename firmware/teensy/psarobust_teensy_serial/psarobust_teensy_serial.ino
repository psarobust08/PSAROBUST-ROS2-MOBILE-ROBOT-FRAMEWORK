/*
 * PSA ROBUST - Teensy 4.1
 * Serial Receiver V1.0
 *
 * ROS 2 -> serial_node -> Teensy 4.1
 * Protocol: vx,vy,wz\n
 * Baud rate: 115200
 *
 * Motor control: NOT IMPLEMENTED
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

constexpr uint32_t BAUD_RATE = 115200;
constexpr size_t BUFFER_SIZE = 64;

char rxBuffer[BUFFER_SIZE];
size_t rxIndex = 0;

float vx = 0.0f;
float vy = 0.0f;
float wz = 0.0f;

void processCommand(const char* data)
{
    float newVx, newVy, newWz;
    char extra;

    int count = sscanf(
        data,
        " %f , %f , %f %c",
        &newVx,
        &newVy,
        &newWz,
        &extra
    );

    if (count != 3)
    {
        Serial.println("ERR,INVALID_FORMAT");
        return;
    }

    if (!isfinite(newVx) ||
        !isfinite(newVy) ||
        !isfinite(newWz))
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

    Serial.println("PSA ROBUST Teensy Serial Receiver V1.0");
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
}

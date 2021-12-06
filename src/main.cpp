#include <Arduino.h>
#include <Servo.h>
// #include <SoftwareSerial.h>

#include "comm.h"
#include "config.h"
#include "touch.h"

char cmdByte = 0;  // for incoming serial data

Servo servo[NUM_LINKS];
int pin[NUM_LINKS] = {7, 9, 10, 11, 12, 13};

// SoftwareSerial touch(0,1);

void setup() {
    Serial.begin(115200);  // opens serial port, sets data rate to 115200 bps
    Serial1.begin(9600);
    for (int i = 0; i < NUM_LINKS; i++) {
        servo[i].attach(pin[i] /*, MIN_BAND, MAX_BAND*/);
    }
}

void loop() {
    // send data only when you receive data:
    if (Serial.available()) {
        // read the incoming byte:
        cmdByte = Serial.read();

        // say what you got:
        Serial.printf("I received: %d | ", cmdByte);

        switch (cmdByte) {
            case CMD_WRITE_DEG:
                Serial.println("CMD: Write DEG, waiting for data...");
                while (!endl()) {
                    servoval_t data = readVal();

                    Serial.printf(" RAW 0x%X\n", data.value);
                    servo[data.servo].write(data.value);
                }
                break;
            case CMD_WRITE_US:
                Serial.println("CMD: Write us, waiting for data...");
                while (!endl()) {
                    servoval_t data = readVal();

                    Serial.printf(" %d us\n", data.value);
                    servo[data.servo].writeMicroseconds(data.value);
                }
                break;
            case CMD_WRITE_MDEG:
                Serial.println("CMD: Write mDeg, waiting for data...");
                while (!endl()) {
                    servoval_t data = readVal();

                    float percent = data.value / 180000;
                    uint16_t us = (percent * (2400 - 544)) + 544;

                    Serial.printf(" %f degs\n", percent * 180);
                    servo[data.servo].writeMicroseconds(us);
                }
                break;
                
            default:
                Serial.print(cmdByte);
                Serial.println("... not a recognized command");
        }
    }
    if (Serial1.available()) {
        uint32_t start = millis();

        while (Serial1.available() < 5) {
            if (millis() - start > TIMEOUT) {
                Serial.printf("Serial 1 timout with %d bytes",
                              Serial.available());
                return;
            }
        }

        uint8_t packet[5];

        for (uint8_t i = 0; i < 5; i++) {
            packet[i] = Serial1.read();
        }

        uint16_t xData = (packet[2] << 8) | packet[1];
        uint16_t yData = (packet[4] << 8) | packet[3];

        Serial.printf("Touch Detected at (%d, %d)\n", xData, yData);
    }
}

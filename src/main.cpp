#include <Arduino.h>
#include <Servo.h>
// #include <SoftwareSerial.h>


#include "invkine.h"
#include "comm.h"
#include "config.h"
#include "touch.h"

char cmdByte = 0;  // for incoming serial data

Servo servo[NUM_LINKS];
int pin[NUM_LINKS] = {7, 9, 10, 11, 12, 13};

// SoftwareSerial touch(0,1);

void setup() {
    Serial.begin(115200);  // opens serial port, sets data rate to 115200 bps
    setupTouch();
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

    checkTouch();

    invkine_setup();

    fvector servo_d(NUM_LINKS);
    servo_d = invKine(5,5);
    Serial.printf("Servo Angles:\n 1: %d\n 2: %d\n 3: %d\n 4: %d\n 5: %d\n 6: %d\n", servo_d[0], servo_d[1], servo_d[2], servo_d[3], servo_d[4], servo_d[5]);
}

// #define __SIMULATE__

#ifdef __SIMULATE__

#include <unistd.h>  // for usleep()
#include <cstdint>
#include <iostream>

using std::cout;
using std::endl;

#define INPUT_PULLUP 2

#define PI (3.14159)

void pinMode(uint8_t, uint8_t) {}

uint8_t digitalRead(uint8_t) { return 0; }

void delay(int t) { }//usleep(t * 1000UL); }
struct {
    void begin(int) {}
    uint8_t read() {
        return 0;
    }
    uint8_t available() {
        return 0;
    }
    template <typename T>
    void print(T x) {
        cout << x;
    }
    template <typename T>
    void println(T x) {
        cout << x << endl;
    }
    template <typename T>
    void printf(T x) {
        cout << x << endl;
    }
    template <typename T, typename D>
    void printf(T x, D d) {
        cout << x << d << endl;
    }
    template <typename T, typename A, typename B>
    void printf(T x, A a, B b) {
        cout << x <<  a << b << endl;
    }
} Serial;

int main() {
    void setup();
    void loop();
    setup();
    for (;;) loop();
}

#endif

#include <Arduino.h>
#include <Servo.h>

#include "comm.h"
#include "config.h"
#include "invkine.h"
#include "touch.h"

#define __VALIDATE__

void checkCommands();

char cmdByte = 0;  // for incoming serial data

Servo servo[NUM_LINKS];
int pin[NUM_LINKS] = {7, 9, 10, 11, 12, 13};

float dx, dy = 0;

// SoftwareSerial touch(0,1);

void setup() {
    setupComm();

    setupTouch();

    invkine_setup();

    for (int i = 0; i < NUM_LINKS; i++) {  // attach all servos
        servo[i].attach(pin[i] /*, MIN_BAND, MAX_BAND*/);
    }
}

void loop() {
    checkCommands();

    checkTouch();

    // static fvector desiredAngles(2);
    // desiredAngles =

    static fvector servo_d(NUM_LINKS);
    static fvector last_d(NUM_LINKS);

    servo_d = invKine(dx, dy);
    // servo_d = invKine(0, 0);

    if (last_d[0] != servo_d[0]) {
        Serial.printf("Servo Angles:");
        for (int i = 0; i < NUM_LINKS; i++) {
            Serial.printf("\t%i: ", i);
            Serial.print(servo_d[i]);
        }
        Serial.println("");
    }

    last_d[0] = servo_d[0];

    for (int i = 0; i < NUM_LINKS; i++) {
        if (i % 2) {
            servo[i].write(90 - servo_d[i]);
        } else {
            servo[i].write(90 + servo_d[i]);
        }
    }
}

void checkCommands() {
    if (Serial.available()) {
        // read the incoming byte:
        cmdByte = Serial.read();

        // say what you got:
        Serial.printf("I received: %d | ", cmdByte);

        servoval_t data;

        switch (cmdByte) {
            case CMD_WRITE_DEG: {
                Serial.println("CMD: Write DEG, waiting for data...");

                data = readVal();

                Serial.printf(" RAW 0x%X\n", data.value);
                servo[data.servo].write(data.value);
            } break;
            case CMD_WRITE_US: {
                Serial.println("CMD: Write us, waiting for data...");

                data = readVal();

                Serial.printf(" %d us\n", data.value);
                servo[data.servo].writeMicroseconds(data.value);
            } break;
            case CMD_WRITE_MDEG: {
                Serial.println("CMD: Write mDeg, waiting for data...");

                data = readVal();

                float percent = data.value / 180000;
                uint16_t us = (percent * (2400 - 544)) + 544;

                Serial.printf(" %f degs\n", percent * 180);
                servo[data.servo].writeMicroseconds(us);
            } break;

            case CMD_WRITE_PLATFORM_ANGLES: {
                Serial.print("CMD: Write Angles (integer degrees); X Angle:");
                uint16_t xIn;
                uint16_t yIn;
                xIn = readASCII();
                Serial.print("Y Angle:");
                yIn = readASCII();

                dx = xIn;
                dy = yIn;

                Serial.printf("Writing to angle (%d,%d)\n", xIn, yIn);
            } break;
            default:
                Serial.print(cmdByte);
                Serial.println("... not a recognized command");
        }
    }
}

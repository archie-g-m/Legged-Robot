#include <Arduino.h>
#include <Servo.h>

#include "comm.h"
#include "config.h"

char cmdByte = 0;  // for incoming serial data

Servo servo[NUM_LINKS];
int pin[NUM_LINKS] = {7,9,10,11,12,13};

void setup() {
    Serial.begin(115200);  // opens serial port, sets data rate to 115200 bps

    for (int i = 0; i < NUM_LINKS; i++) {
        servo[i].attach(pin[i]/*, MIN_BAND, MAX_BAND*/);
    }
}

void loop() {
    // send data only when you receive data:
    if (Serial.available() > 0) {
        // read the incoming byte:
        cmdByte = Serial.read();

        // say what you got:
        Serial.print("I received: ");
        Serial.print(cmdByte, DEC);

        switch (cmdByte) {
            case CMD_WRITE_RAW:
                Serial.println(" | CMD: Write RAW, waiting for data...");
                while (!endl()) {
                    waitForByte();
                    uint8_t servoNum = Serial.read() - 48;
                    Serial.printf("Setting Servo %d to ...", servoNum);
                    uint16_t servoVal;
                    // readInt16(&servoVal);
                    char numbers[5];
                    numbers[4] = '\0';

                    for (int i = 0; i < 4; i++){
                        waitForByte();
                        numbers[i] = Serial.read();
                    }

                    servoVal = atoi(numbers);

                    Serial.printf(" RAW 0x%X\n", servoVal);
                    servo[servoNum].write(servoVal);
                }
                default:
                Serial.print(cmdByte);
                Serial.println(" not a recognized command");
        }
    }
}

void writeRaw(uint16_t servo, uint16_t value) {}

uint16_t processValue(uint16_t value) { return 1; }

void read() {}

// void calibrate() { for (uint16_t i = 0; i < NUM_LINKS) }

// void recv(void *buffer, uint8_t size) {
//     for (uint8_t i = 0; i < size; i) {
//         while (!Serial.available()) {
//         }
//         *buffer = Serial.read();
//     }
// }
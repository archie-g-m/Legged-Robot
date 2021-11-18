#include "comm.h"

void waitForByte() { waitForBytes(1); }

void waitForBytes(uint8_t numBytes) {
    while (Serial.available() < numBytes) {
        delay(1);
    }
}

bool readNum(void *p, uint8_t size) {
    // long start = millis();
    // while (Serial.available() < size) {
    //     if (millis() - start > TIMEOUT) return false;
    // }

    uint8_t *addr = (uint8_t *)p;

    for (uint8_t i = 0; i < size; i++) {
        waitForByte();
        addr[i] = Serial.read();  // fill buffer with next N bytes of data from
                                  // Serial.read
    }

    return true;
}

bool readFloat(float *p) { return readNum(p, 4); }

bool readInt16(uint16_t *p) { return readNum(p, 2); }

bool endl() { return (Serial.peek() == '\n'); }
#include "comm.h"

void setupComm(){
    Serial.begin(115200);  // opens serial port, sets data rate to 115200 bps
}

void waitForByte() { waitForBytes(1); }

void waitForBytes(uint8_t numBytes) {
    while (Serial.available() < numBytes) {
        delay(1);
    }
}

bool readNum(void *p, uint8_t size) {
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

servoval_t readVal() {
    Serial.print("Select Servo: ");
    waitForByte();                          // wait for a servo #
    uint8_t servoNum = Serial.read() - 48;  // convert ascii to int value
    Serial.printf(" %d to value: ", servoNum);

    uint16_t val;

    if (DATA_MODE == USE_ASCII) {
        val = readASCII();
    } else {
        val = readInt16(&val);
    }

    return {servoNum, val};
}

uint16_t readASCII() {
    char buffer[10] = {0xFF};
    uint8_t index = 0;

    do {
        waitForByte();
        buffer[index] = Serial.read();
        Serial.printf(" %c ", buffer[index]);
        index++;
    } while (buffer[index - 1] != '\0');

    return atoi(buffer);
}

bool endl() {
    // Serial.println("endl?");
    // waitForByte();
    // Serial.printf("%s\n", Serial.peek() == '\n' ? "yup" : "nope");
    return (Serial.peek() == '\n');
}

void printBits(uint64_t data, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        Serial.printf("%d ", (data >> i) & 1);
    }
}
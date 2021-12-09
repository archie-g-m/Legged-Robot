#include <Arduino.h>

#define TIMEOUT (250)  // timemout duration in ms

#define USE_ASCII (0)
#define USE_DATA (1)

#define DATA_MODE (USE_ASCII)

typedef struct servoval_t {
    uint8_t servo;  // servo num
    uint16_t value;
} servoval_t;

void setupComm();

servoval_t readVal();

void waitForByte();

void waitForBytes(uint8_t numBytes);

bool readFloat(float* p);

bool readInt16(uint16_t* p);

bool readNum(void* p, uint8_t size);

uint16_t readASCII();

bool endl();

void printBits(uint64_t data, uint8_t size);

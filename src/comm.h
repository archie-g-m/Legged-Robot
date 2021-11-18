#include <Arduino.h>

#define TIMEOUT (500) // timemout duration in ms

void waitForByte();

void waitForBytes(uint8_t numBytes);

bool readFloat(float* p);

bool readInt16(uint16_t* p);

bool readNum(void* p, uint8_t size);

bool endl();

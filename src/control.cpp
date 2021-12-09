#include "control.h"

const float Kp = 1.0f;
const float Ki = 0.0f;
const float Kd = 0.0f;

float PID(uint16_t target, uint16_t current) {
    int16_t error = target - current;

    static uint16_t errorSum = 0;
    errorSum += error;

    static uint16_t errorDiff;
    static int16_t lastError = 0;
    errorDiff = error - lastError;
    lastError = error;

    return Kp * error + Ki * errorSum + Kd * errorDiff;
}
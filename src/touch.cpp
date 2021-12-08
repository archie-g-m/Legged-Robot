#include "touch.h"
#include "comm.h"

coord_t setupTouch(){
    Serial1.begin(9600);
}

coord_t checkTouch(){

    static uint16_t xData;
    static uint16_t yData;

    if (Serial1.available()) {
        uint32_t start = millis();

        while (Serial1.available() < 5) {
            if (millis() - start > TIMEOUT) {
                Serial.printf("Serial 1 timout with %d bytes",
                              Serial.available());
                return {xData, yData};
            }
        }

        uint8_t packet[5];

        for (uint8_t i = 0; i < 5; i++) {
            packet[i] = Serial1.read();
        }

        yData = (packet[2] << 8) | packet[1];
        xData = (packet[4] << 8) | packet[3];

        Serial.printf("Touch Detected at (%d, %d)\n", xData, yData);
    }

    return {xData, yData};
}
#include "touch.h"
#include "comm.h"

coord_t touchPos = {0,0};

void setupTouch(){
    Serial1.begin(9600);
}

/**
 * @brief Checks the touch sensor UART for sent data, updating the store touch Pos. 
 * 
 * @return true if a new reading is available
 * @return false if a new reading is not available
 */
bool checkTouch(){

    if (Serial1.available()) {
        uint32_t start = millis();

        while (Serial1.available() < 5) {
            if (millis() - start > TIMEOUT) {
                Serial.printf("Serial 1 timout with %d bytes",
                              Serial.available());
                Serial1.flush();
                return false; // if comm times out; flush buffer and return false
            }
        }

        uint8_t packet[5];

        for (uint8_t i = 0; i < 5; i++) {
            packet[i] = Serial1.read();
        }

        uint16_t yData = (packet[2] << 8) | packet[1];
        uint16_t xData = (packet[4] << 8) | packet[3];

        Serial.printf("Touch Detected at (%d, %d)\n", xData, yData);

        touchPos = {yData, xData};

        return true;
    }

    return false;
}
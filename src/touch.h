#include <Arduino.h>

typedef struct coord_t {
    uint16_t x, y;
} coord_t;

extern coord_t touchPos;

#define CMD_TOUCH_ENABLE (0x12)     // Enable TOUCH reporting
#define CMD_TOUCH_DISABLE (0x13)    // Disable TOUCH reporting
#define CMD_CALIBRATE (0x14)        // Execute Calibrate routine
#define CMD_REG_READ (0x20)         // Read register(s)
#define CMD_REG_WRITE (0x21)        // Write register(s)
#define CMD_EE_READ (0x28)          // Read EE location(s)
#define CMD_EE_WRITE (0x29)         // Write EE location(s)
#define CMD_EE_READ_PARAMS (0x2B)   // Read parameter set (from EE to RAM)
#define CMD_EE_WRITE_PARAMS (0x23)  // Write parameter set (from RAM to EE)

#define STATUS_OK (0x00)            // No error
#define STATUS_UNRECOGNIZED (0x01)  // Unrecognized command
#define STATUS_TIMEOUT (0x04)       // Packet time out
#define STATUS_EEPARAMS_ERR (0x05)  // Error reading EEPROM parameters
#define STATUS_CAL_CANCEL (0xFC)    // Calibration sequence cancelled


void setupTouch();
bool checkTouch();
#include <Arduino.h>

#define VR_x 26
#define VR_y 25

#define Zero_x 1955
#define Zero_y 1899

#define THRESHOLD 100 
void setup() {
  Serial.begin(115200);
  pinMode(VR_x, INPUT);
  pinMode(VR_y, INPUT);
}

void publish_joystick(int x_normal, int y_normal) {

}

void loop() {
  int x_reading = analogRead(VR_x);
  int y_reading = analogRead(VR_y);

  int x_normal = x_reading - Zero_x;
  int y_normal = y_reading - Zero_y;

  if (abs(x_normal) > 100 or abs(y_normal) > 100) {
    publish_joystick(x_normal, y_normal);
  }
  sleep(1);
}


// 4095 0
// x: 1955
// y: 1899
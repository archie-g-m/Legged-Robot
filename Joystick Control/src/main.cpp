#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>

#define USE_USBCON

#define VR_X 25
#define VR_Y 26

#define ZERO_X 1955
#define ZERO_Y 1899

#define THRESHOLD 100 

ros::NodeHandle  nh;
geometry_msgs::Vector3 command;
ros::Publisher walk_pub("/robot_walk", &command);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);

  pinMode(VR_X, INPUT);
  pinMode(VR_Y, INPUT);

  nh.initNode();
  nh.advertise(walk_pub);
}

void publish_joystick(int x_normal, int y_normal) {
  command.x = x_normal;
  command.y = y_normal;
  command.z = 1;
  walk_pub.publish(&command);
}

void loop() {
  int x_reading = analogRead(VR_X);
  int y_reading = analogRead(VR_Y);

  int x_normal = x_reading - ZERO_X;
  int y_normal = y_reading - ZERO_Y;

  if (abs(x_normal) > THRESHOLD or abs(y_normal) > THRESHOLD) {
    x_normal = map(x_normal, -1995, 2140, -10, 10);
    y_normal = map(y_normal, -1899, 2196, -10, 10);
    publish_joystick(x_normal, y_normal);
  } else 
  {
    // publish_joystick(0, 0);
  }
  nh.spinOnce();
  delay(300);
}

int main() {
  setup();
  
  while(true) {
    loop();
  }
  return 0;
}


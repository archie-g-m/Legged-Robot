#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// which pins the sensor is hooked up to
static const int leftPin = ; 
static const int rightPin = ; 
static const int centerPin = ; 

ros::NodeHandle nh;
geometry_msgs::Twist command;
ros::Publisher line_pub("/line_follower", &command);

void setup(){
    Serial.begin(115200);
    nh.getHardware()->setBaud(115200);

    //set up the pins
    pinMode(leftPin, INPUT);
    pinMode(rightPin, INPUT);
    pinMode(centerPin, INPUT);

    Twist twist;

    nh.initNode();
    nh.advertise(line_pub);
}

void publish_line(float lin_x, float ang_z) {
    command.linear.x = lin_x; //whatever value we need
    command.linear.y = 0; //never changes
    command.linear.z = 0; //never changes
    command.angular.x = 0; //never changes
    command.angular.y = 0; //never changes
    command.angular.z = ang_z;

    line_pub.publish(&command);
}

void loop() {
    float left_read = analogRead(leftPin);
    float right_read = analogRead(rightPin);
    float center_read = analogRead(centerPin);

    // need to set the actual values for comparison tape/no tape
    float val_des = 500;
    if (right_read > val_des) {
        //adjust heading toward the left
        float error = right_read - val_des;
        float lin_X = 0.2; //whatever forward velocity we are using
        float ang_Z = error/100; //changes the forward direction
        publish_line(lin_X, ang_Z);
    }
    else if (left_read > val_des) {
        // adjust heading toward the right
        float error = left_read - val_des;
        float lin_X = 0.2; //whatever forward velocity we are using
        float ang_Z = error/100; //changes the forward direction
        publish_line(lin_X, ang_Z);
    }
    else {
        //continue forward, no alterations
        float lin_X = 0.2; //whatever forward velocity we are using
        float ang_Z = 0;
        publish_line(lin_X, ang_Z);
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




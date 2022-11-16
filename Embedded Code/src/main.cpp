#include <unordered_map>
#include <string>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

#define USMIN 600
#define USMAX 2400

void writeServoDeg(int servo, int deg);
void servo_cb(const sensor_msgs::JointState& cmd_msg);

std_msgs::String str_msg;

ros::NodeHandle  nh;
ros::Subscriber<sensor_msgs::JointState> sub("/servo", servo_cb);
ros::Publisher chatter("/chatter", &str_msg);

std::unordered_map<std::string, int> joint_map;

const int port_map[12] = {
    0, 1, 2, 
    4, 5, 6, 
    8, 9, 10, 
    12, 13, 14
    };
const int zero_us[12] = {
    2000, 1500, 1400, 
    2000, 1500, 1400, 
    2000, 1500, 1400, 
    2000, 1500, 1400,
};

void writeServoDeg(int servo, int deg) {
    int us = (deg*(USMAX-USMIN)/180) + zero_us[servo];
    int port = port_map[servo];
    pwm.writeMicroseconds(port, us);
}

void servo_cb(const sensor_msgs::JointState& cmd_msg){
    writeServoDeg(joint_map[cmd_msg.name[0]], cmd_msg.position[0]);
    // str_msg.data = ("Setting Joint " + (std::string)cmd_msg.name[0] +" (" + std::to_string(port_map[joint_map[cmd_msg.name[0]]]) + ") to "  + std::to_string(cmd_msg.position[0]) +  " degrees").c_str();
    // chatter.publish(&str_msg);
}

void setup() {
    Serial.begin(115200);
    nh.getHardware()->setBaud(115200);

    // Leg 1
    joint_map["a1"] = 0;
    joint_map["k1"] = 1;
    joint_map["h1"] = 2;
    // Leg 2
    joint_map["a2"] = 3;
    joint_map["k2"] = 4;
    joint_map["h2"] = 5;
    // Leg 3
    joint_map["a3"] = 6;
    joint_map["k3"] = 7;
    joint_map["h3"] = 8;
    // Leg 4
    joint_map["a4"] = 9;
    joint_map["k4"] = 10;
    joint_map["h4"] = 11;

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);

    pwm.begin();
    /*
     * In theory the internal oscillator (clock) is 25MHz but it really isn't
     * that precise. You can 'calibrate' this by tweaking this number until
     * you get the PWM update frequency you're expecting!
     * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     * is used for calculating things like writeMicroseconds()
     * Analog servos run at ~50 Hz updates, It is importaint to use an
     * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
     *    the I2C PCA9685 chip you are setting the value for.
     * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     *    expected value (50Hz for most ESCs)
     * Setting the value here is specific to each individual I2C PCA9685 chip and
     * affects the calculations for the PWM update frequency.
     * Failure to correctly set the int.osc value will cause unexpected PWM results
     */
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    

    int home_vals[3] = {-120, 10, 0};
    for (int i = 0; i < 12; i++) {
        writeServoDeg(i, home_vals[i%3]);
    }

    delay(10);
}

void loop() {
    nh.spinOnce();
    delay(1);
}

int main() {
    setup();
    while (true) {
        loop();
    }
    return 0;
}
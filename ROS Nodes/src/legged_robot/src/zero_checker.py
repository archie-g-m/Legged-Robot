#!/usr/bin/env python3

import rospy
import sensor_msgs.msg

if __name__ == "__main__":
    rospy.init_node("zero_checker")
    servo_pub = rospy.Publisher("/servo", sensor_msgs.msg.JointState, queue_size=16)
    
    joints = ["a1","a2","a3","a4",
              "k1","k2","k3","k4",
              "h1","h2","h3","h4",]
    
    zero_angle = [-900,0,0,0,
                  -10,-12,-10,-10,
                  7,5,2,10]
    
    msg = sensor_msgs.msg.JointState()
    for i in range(12):
        msg.name = [joints[i]]
        msg.position = [zero_angle[i]]
        servo_pub.publish(msg)
        rospy.sleep(0.1)
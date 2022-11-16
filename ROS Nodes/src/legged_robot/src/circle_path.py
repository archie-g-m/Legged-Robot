#!/usr/bin/env python3
import rospy
import numpy as np
import geometry_msgs.msg

pi = np.pi

if __name__ == "__main__":
    rospy.init_node("circle_path")
    pose_pub = rospy.Publisher("robot_pose", geometry_msgs.msg.Pose, queue_size=16)
    d0 = 50
    dz = 50 #mm
    v_z = 10 #mm/s
    s_per_motion = dz/v_z
    pts_per_sec = 12
    pts_per_motion = int(s_per_motion*pts_per_sec)
    print(pts_per_motion)
            
    msg = geometry_msgs.msg.Pose()
    msg.position.z = 50
    for i in range(3):
        for t in range(pts_per_motion):
            dt = 1/pts_per_sec
            msg.position.z = d0+(v_z * (t/pts_per_sec));
            pose_pub.publish(msg)
            rospy.sleep(1/pts_per_sec)
            
        for t in range(pts_per_motion):
            msg.position.z = d0+dz-(v_z * (t/pts_per_sec));
            pose_pub.publish(msg)
            rospy.sleep(1/pts_per_sec)

        
        
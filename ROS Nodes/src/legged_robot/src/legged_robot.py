#!/usr/bin/env python3

import rospy
import time
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg

pi = np.pi
euler_convention = "XYZ"
 
theta = np.array([pi/4, 3*pi/4, -pi/4, -3*pi/4]) #radians
n_legs = np.size(theta)

l1_m = 40 #mm
l2_m = 75 #mm
l3_m = 100 #mm

r_p = 56.57 #mm
w_b = 200
h_b = 150 #mm

s = r_p * np.vstack((np.cos(theta), np.sin(theta), np.zeros_like(theta)))
u = np.array([[w_b/2, -w_b/2, w_b/2, -w_b/2],
              [h_b/2, h_b/2, -h_b/2, -h_b/2],
              [0,0,0,0]])
print(u)
# u = w_b/2 * np.vstack((np.cos(theta), np.sin(theta), np.zeros_like(theta)))

# print(u)


joint_names = [["h1", "k1", "a1"],
               ["h3", "k3", "a3"],
               ["h2", "k2", "a2"],
               ["h4", "k4", "a4"],]

hip_offset = [-45, 45, 45, -45] #deg
hip_coeffs = [1, 1, 1, 1]

class LeggedRobot:
    def __init__(self):
        rospy.init_node("legged_robot")
        self.pose_sub = rospy.Subscriber("robot_pose", geometry_msgs.msg.Pose, self.ik, queue_size=16)
        self.servo_pub = rospy.Publisher("servo", sensor_msgs.msg.JointState, queue_size=16)



    def ik(self, msg: geometry_msgs.msg.Pose):
        start = time.time()
        O = np.array([msg.position.x,msg.position.y,msg.position.z])
        R = quat2R(msg.orientation, euler_convention)
        alpha  = np.empty_like(theta)
        beta  = np.empty_like(theta)
        gamma  = np.empty_like(theta)
        rho = np.empty_like(theta)
        psi = np.empty_like(theta)
        for i in range(n_legs):
            l = O + R.dot(s[:,i]) - u[:,i]
            alpha[i] = np.arctan(l[1] / l[0])
            
            l_1 = l1_m * np.array([np.cos(alpha[i]), np.sin(alpha[i]), 0]) * np.array([(-1)**(i), (-1)**(i), 1])

            s_2 = s[:,i] + l_1
            l_prime = O + R.dot(s_2) - u[:,i]
            # print(l_prime)
            
            l_primem = np.linalg.norm(l_prime, 2)
            
            gamma[i] = pi-np.arccos((l2_m**2 + l3_m**2 - l_primem**2) / (2 * l2_m * l3_m))
            
            h = l[2]
            
            h_prime = l_prime[2]
            
            psi[i] = np.arcsin((h_prime-h)/l1_m)
            rho[i] = np.arctan(h_prime / np.sqrt((l_prime[0]**2) + (l_prime[1]**2)))
            
            beta[i] = np.arccos((l2_m**2 + l_primem**2 - l3_m**2) / (2*l2_m*l_primem)) - (rho[i] + psi[i])
            
            
        print(f"Joint Angles for Pose: [{msg.position.x}, {msg.position.y}, {msg.position.z}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}")
        print(f"alpha = {alpha}")
        print(f"beta = {beta}")
        print(f"gamma = {gamma}")
        print(f"rho = {rho}")
        print(f"psi = {psi}")
        print("")

        angles = np.stack([alpha, beta, gamma])

        for i in range(n_legs):
            for j in range(3):
                joint_msg = sensor_msgs.msg.JointState()
                joint_msg.name = [joint_names[i][j]]
                if j == 0:
                    joint_msg.position = [hip_coeffs[i]*((angles[j,i]*180/pi)+hip_offset[i])]
                elif j == 1:
                    joint_msg.position = [angles[j,i]*180/pi]           
                elif j == 2: 
                    joint_msg.position = [-1*(angles[j,i]*180/pi)]     
                self.servo_pub.publish(joint_msg) 
        # print(f"took: {time.time()-start}s")
        rospy.sleep(0.01)
        return
            

def quat2R(euler:geometry_msgs.msg.Quaternion, convention:str):
    a = euler.x
    b = euler.y
    c = euler.z
    
    angles = [a,b,c]
    
    def Rx(angle):
        return np.array([[1,0,0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])
    
    def Ry(angle):
        return np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0,1,0],
                         [-np.sin(angle),0,np.cos(angle)]])
    
    def Rz(angle):
        return np.array([[np.cos(angle), -np.sin(angle), 0],
                        [np.sin(angle), np.cos(angle), 0],
                        [0,0,1]])
    
    rot_map = {"X": Rx, "Y": Ry, "Z": Rz}
    
    R = np.eye(3)
    for i, ang in enumerate(angles):
        R = R.dot(rot_map[convention[i]](ang))
    
    return R

if __name__ == '__main__':
    LeggedRobot()
    rospy.sleep(1)
    rospy.spin()
    pass
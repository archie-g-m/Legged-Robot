#!/usr/bin/env python3
import rospy
import time
import math
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg
import legged_robot.msg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
from plot_utilities import plotLine, plotLine2, plotPoint

pi = np.pi
euler_convention = "XYZ"

theta = np.array([pi/4, 3*pi/4, -pi/4, -3*pi/4])  # radians
n_legs = np.size(theta)

l1_m = 40  # mm
l2_m = 75  # mm
l3_m = 100  # mm

r_p = 56.57  # mm
w_b = 200
h_b = 150  # mm

s = r_p * np.vstack((np.cos(theta), np.sin(theta), np.zeros_like(theta)))
u = np.array([[w_b/2, -w_b/2, w_b/2, -w_b/2],
              [h_b/2, h_b/2, -h_b/2, -h_b/2],
              [0, 0, 0, 0]])

body_loop = [1,3,0,2]

leg_colors = [[1,0,1],[0,1,1],[1,0,0],[0,0,1]];

joint_names = [["h1", "k1", "a1"],
               ["h3", "k3", "a3"],
               ["h2", "k2", "a2"],
               ["h4", "k4", "a4"], ]

hip_offset = [-45, 45, 45, -45]  # deg
hip_coeffs = [1, 1, 1, 1]


class LeggedRobot:
    def __init__(self, plot=False):
        rospy.init_node("legged_robot")
        self.pose_sub = rospy.Subscriber(
            "robot_pose", geometry_msgs.msg.Pose, self.ik, queue_size=16)
        self.walk_sub = rospy.Subscriber(
            "robot_walk", legged_robot.msg.walk, self.walk, queue_size=16)
        self.servo_pub = rospy.Publisher(
            "servo", sensor_msgs.msg.JointState, queue_size=16)
        
        self.alpha = np.zeros_like(theta)
        self.beta = np.zeros_like(theta)
        self.gamma = np.zeros_like(theta)
        self.rho = np.zeros_like(theta)
        self.psi = np.zeros_like(theta)
        self.O = np.array([0,0,100]) #mm
        self.R = np.eye(3)
        
        if plot:
            self.fig = plt.figure()
            self.ax = mplot3d.axes3d.Axes3D(self.fig)
            ani = animation.FuncAnimation(self.fig, self.plot_robot, interval=int(1000/30))
            plt.show(block=True)
        else:
            rospy.spin()
        
    def plot_robot(self, *args):
        self.ax.cla()
        for i in range(n_legs):
            s_g = self.O + np.dot(self.R, s[:,i])
            l1 = s_g + (np.dot(self.R, l1_m * np.stack([np.cos(self.alpha[i]), 
                                                    np.sin(self.alpha[i]), 
                                                    0])) * np.array([(-1)**(i), (-1)**(i), 1]))
            l2 = l1 + (l2_m * np.stack([np.cos(self.alpha[i]) * np.cos(self.beta[i] + self.psi[i]), 
                                        np.sin(self.alpha[i]) * np.cos(self.beta[i] + self.psi[i]), 
                                        np.sin(self.beta[i] + self.psi[i])]) * np.array([(-1)**(i), (-1)**(i), 1]))
            lam = np.pi - (np.pi-self.gamma[i]) - (self.beta[i]+self.psi[i]+self.rho[i])
            l3 = l2 + (l3_m * np.stack([np.cos(self.alpha[i]) * np.cos(-lam - self.rho[i]), 
                                        np.sin(self.alpha[i]) * np.cos(-lam - self.rho[i]), 
                                        np.sin(-lam - self.rho[i])]) * np.array([(-1)**(i), (-1)**(i), 1]))
            #Plot Foot Base Reference Points
            plotPoint(self.ax, u[:,i], leg_colors[i])
            #Plot Foot End Effector Point
            plotPoint(self.ax, l3, leg_colors[i])
            #Plot First Link
            plotLine2(self.ax, s_g, l1, leg_colors[i])
            #Plot Second Link
            plotLine2(self.ax, l1, l2, leg_colors[i])
            #Plot Third Link
            plotLine2(self.ax, l2, l3, leg_colors[i])
            #Plot Top Platform
            plotLine2(self.ax, s_g, self.O + np.dot(self.R, s[:,body_loop[i]]), [0,0,0])
        self.ax.set_xlim3d(-300,300)
        self.ax.set_ylim3d(-300,300)
        self.ax.set_zlim3d(0,600)

    def walk(self, msg: legged_robot.msg.walk):        
        return

    def ik(self, msg: geometry_msgs.msg.Pose):
        start = time.time()
        self.O = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.R = quat2R(msg.orientation, euler_convention)
        for i in range(n_legs):
            self.alpha[i], self.beta[i], self.gamma[i], self.rho[i], self.psi[i] = self.ik_serial(s[:,i], u[:,i])

        # print(f"Joint Angles for Pose: [{msg.position.x}, {msg.position.y}, {msg.position.z}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}")
        # print(f"self.alpha = {self.alpha}")
        # print(f"self.beta = {self.beta}")
        # print(f"self.gamma = {self.gamma}")
        # print(f"self.rho = {self.rho}")
        # print(f"self.psi = {self.psi}")
        # print("")
        
        angles = np.stack([self.alpha, self.beta, self.gamma])
        
        self.publish_legs()
        # print(f"took: {time.time()-start}s")
        rospy.sleep(0.01)
        return

    def ik_serial(self, s, u):
        l = self.O + self.R.dot(s) - u
        
        alpha = np.arctan(l[1] / l[0])

        l_1 = l1_m * np.array([np.cos(alpha), np.sin(alpha), 0]) * np.array([(-1)**(i), (-1)**(i), 1])

        s_2 = s + l_1
        l_prime = self.O + self.R.dot(s_2) - u

        l_primem = np.linalg.norm(l_prime, 2)

        gamma = pi-np.arccos((l2_m**2 + l3_m **2 - l_primem**2) / (2 * l2_m * l3_m))

        h = l[2]

        h_prime = l_prime[2]

        psi = np.arcsin((h_prime-h)/l1_m)
        rho = np.arctan(h_prime / np.sqrt((l_prime[0]**2) + (l_prime[1]**2)))

        beta = np.arccos((l2_m**2 + l_primem**2 - l3_m**2) / (2*l2_m*l_primem)) - (rho + psi)
        
        return alpha, beta, gamma, rho, psi
    
    def ik_serial2(self, x, y, z):

        w = self.getDist(x, y)
        a = w - l1_m
        b = self.getDist(a, z)
        self.psi = math.atan(-z/a)

        # print(w)
        # print(a)
        # print(b)
        # print(self.psi)

        theta1 = math.atan(y/x)
        theta2 = self.getLOC(l2_m, b, l3_m) - self.psi
        theta3 = np.pi - self.getLOC(l2_m, l3_m, b)

        return theta1, theta2, theta3

    def getDist(self, a, b):
        return math.sqrt(a**2 + b**2)

    def getLOC(self, a, b, c):
        expr = (a**2 + b**2 - c**2)/(2*a*b)
        return math.acos(expr)
    
    def publish_legs(self):
        for i in range(n_legs):
            for j in range(3):
                joint_msg = sensor_msgs.msg.JointState()
                joint_msg.name = [joint_names[i][j]]
                if j == 0:
                    joint_msg.position = [hip_coeffs[i] * ((self.alpha[i]*180/pi)+hip_offset[i])]
                elif j == 1:
                    joint_msg.position = [self.beta[i]*180/pi]
                elif j == 2:
                    joint_msg.position = [-1*(self.gamma[i]*180/pi)]
                self.servo_pub.publish(joint_msg)

def quat2R(euler: geometry_msgs.msg.Quaternion, convention: str):
    a = euler.x
    b = euler.y
    c = euler.z

    angles = [a, b, c]

    def Rx(angle):
        return np.array([[1, 0, 0],
                        [0, np.cos(angle), -np.sin(angle)],
                        [0, np.sin(angle), np.cos(angle)]])

    def Ry(angle):
        return np.array([[np.cos(angle), 0, np.sin(angle)],
                        [0, 1, 0],
                        [-np.sin(angle), 0, np.cos(angle)]])

    def Rz(angle):
        return np.array([[np.cos(angle), -np.sin(angle), 0],
                        [np.sin(angle), np.cos(angle), 0],
                        [0, 0, 1]])

    rot_map = {"X": Rx, "Y": Ry, "Z": Rz}

    R = np.eye(3)
    for i, ang in enumerate(angles):
        R = R.dot(rot_map[convention[i]](ang))

    return R


if __name__ == '__main__':
    LeggedRobot(plot=True)
    # rospy.sleep(1)
    # rospy.spin()
    pass

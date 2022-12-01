#!/usr/bin/env python3
import rospy
import time
import math
import numpy as np
import sensor_msgs.msg
import geometry_msgs.msg
# import legged_robot.msg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
from plot_utilities import plotLine, plotLine2, plotPoint

pi = np.pi
euler_convention = "XYZ"

theta = np.array([pi/4, 3*pi/4, -pi/4, -3*pi/4])  # radians
n_legs = np.size(theta)

#Walking Constraints
beta = 0.75
H =  50 #mm
L_g = 50 #mm

#Construction Parameters
l1_m = 40  # mm
l2_m = 75  # mm
l3_m = 100 # mm

r_p = 56.57# mm
w_b = 200  # mm
h_b = 200  # mm

sway_w = w_b/8
sway_h = h_b/8

s = r_p * np.vstack((np.cos(theta), np.sin(theta), np.zeros_like(theta)))
u = np.array([[w_b/2, -w_b/2, w_b/2, -w_b/2],
              [h_b/2, h_b/2, -h_b/2, -h_b/2],
              [0, 0, 0, 0]])

body_loop = [1,3,0,2]

leg_colors = [[1,0,1],[0,1,1],[1,0,0],[0,0,1]]

joint_names = [["h1", "k1", "a1"],
               ["h2", "k2", "a2"],
               ["h3", "k3", "a3"],
               ["h4", "k4", "a4"], ]

joint_offsets = [[0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, -0, 5], ]

hip_offset = [-45, 45, 45, -45]  # deg
hip_coeffs = [1, 1, 1, 1]



class LeggedRobot:
    def __init__(self, plot=False):
        rospy.init_node("legged_robot")
        self.pose_sub = rospy.Subscriber(
            "robot_pose", geometry_msgs.msg.Pose, self.ik_paralell, queue_size=16)
        self.walk_sub = rospy.Subscriber(
            "robot_walk", geometry_msgs.msg.Vector3, self.walk, queue_size=16)
        self.servo_pub = rospy.Publisher(
            "servo", sensor_msgs.msg.JointState, queue_size=16)
        
        self.alpha = np.zeros_like(theta)
        self.beta = np.zeros_like(theta)
        self.gamma = np.zeros_like(theta)
        self.rho = np.zeros_like(theta)
        self.psi = np.zeros_like(theta)
        self.P = np.array([0,0,75])
        self.O = np.array([0,0,0]) #mm
        self.R = np.eye(3)
        rospy.sleep(1)
        self.ik_paralell(None)
        
        self.support_end, self.transfer_end = self.getPhases(beta, 1)
        self.transfer_end[[0,2]] = self.transfer_end[[2,0]]
        self.support_end[[0,2]] = self.support_end[[2,0]]
        print(self.support_end)
        print(self.transfer_end)
        
        self.v_start = time.time()
        self.dir = 0
        self.speed = 0
        self.travel_time = 0
        if plot:
            self.fig = plt.figure()
            self.ax = mplot3d.axes3d.Axes3D(self.fig)
            ani = animation.FuncAnimation(self.fig, self.plot_robot, interval=int(1000/30))
            plt.show(block=True)
        else:
            rospy.spin()
        
    def plot_robot(self, *args):
        self.ax.cla()
        support_polygon = None
        for i in range(n_legs):
            s_g = self.O + self.P + self.R.dot(s[:,i])
            l1 = s_g + (self.R.dot(l1_m * np.stack([np.cos(self.alpha[i]), 
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
            # plotPoint(self.ax, l3, leg_colors[i])
            #Plot First Link
            plotLine2(self.ax, s_g, l1, leg_colors[i])
            #Plot Second Link
            plotLine2(self.ax, l1, l2, leg_colors[i])
            #Plot Third Link
            plotLine2(self.ax, l2, l3, leg_colors[i])
            #Plot Top Platform
            plotLine2(self.ax, s_g, self.O + self.P + self.R.dot(s[:,body_loop[i]]), [0,0,0])
            # Plot Support Polygon
            if int(l3[2]) == 0:
                if support_polygon is None:
                    support_polygon = l3
                else:
                    support_polygon = np.vstack((support_polygon, l3))
        support_polygon = np.vstack((support_polygon, support_polygon[0,:]))
        plotLine(self.ax, support_polygon, [1,.25,0])
        plotPoint(self.ax, self.O + np.array([self.P[0], self.P[1], 0]), [0,0,0])    
        self.ax.set_xlim3d(-300,300)
        self.ax.set_ylim3d(-300,300)
        self.ax.set_zlim3d(0,600)

    def walk(self, msg: geometry_msgs.msg.Vector3): 
        self.speed = np.sqrt(msg.x**2 + msg.y**2)
        self.dir = np.arctan2(msg.x, msg.y) 
        self.v_start = time.time()
        self.travel_time = msg.z
        D = self.speed*self.travel_time
        L = D/np.ceil(D/L_g)
        T = L/self.speed
        print(f"Moving {D/L}x{L}mm ({T}s) strides")
        t = 0
        last_time = 0
        rel_t = 0
        
        stepped = [False, False, False, False]
      
        while(t < self.travel_time):
            t = time.time()-self.v_start
            dt = t - last_time
            rel_t += dt
            
            distance = self.speed*dt
            # print(distance)
            self.P = np.array([sway_w*np.cos(2*np.pi*(-rel_t/T + 1/4)) + (np.cos(self.dir)*distance), sway_h*np.sin(2*np.pi*(-rel_t/T + 1/4)) + (np.sin(self.dir)*distance), self.P[2]])
            # print(self.P)
            self.O = self.O + np.array([np.cos(self.dir), np.sin(self.dir), 0])*distance
            
            # print(self.O)
            
            # Calculate the relative time within the stride
            # print(rel_t)
            for i in range(n_legs):
               
                #When the foot is placed down update the ground point
                if rel_t >= (self.transfer_end[i]*T) and not stepped[i]:
                    # print(f"Leg {i} completed STEP\n")
                    u[:,i] = u[:,i] + L*np.array([np.cos(self.dir), np.sin(self.dir), 0])
                    stepped[i] = True
                
                # if i == 2:
                #     print(rel_t)
                #if leg is in transfer phase
                if  (self.support_end[i]*T) < rel_t and rel_t < (self.transfer_end[i]*T):
                    #Get the xyz coordinates of the foot w.r.t foot trajectory frame
                    x,y,z = self.getFootTraj(rel_t-(self.support_end[i]*T), (1-beta)*T, H, L)
                    #Convert xyz coordinates of foot to Global Frame
                    u_g = ([x+(L/2*np.cos(self.dir)), y+(L/2*np.sin(self.dir)), z] + u[:, i])
                #if leg is in support phase
                else:
                    # print("PARALELL\n")
                    u_g = u[:, i]
                
                self.alpha[i], self.beta[i], self.gamma[i], self.rho[i], self.psi[i] = self.ik_serial(s[:,i], u_g, i)

            if rel_t > T:
                rel_t-=T
                stepped = [False, False, False, False]
                    
            
            self.publish_legs()
            time.sleep(0.1)
            last_time = t
        
    def ik_paralell(self, msg: geometry_msgs.msg.Pose):
        start = time.time()
        if msg is not None:
            self.P = np.array([msg.position.x, msg.position.y, msg.position.z])
            self.R = quat2R(msg.orientation, euler_convention)
        for i in range(n_legs):
            self.alpha[i], self.beta[i], self.gamma[i], self.rho[i], self.psi[i] = self.ik_serial(s[:,i], u[:,i], i)

        # print(f"Joint Angles for Pose: [{msg.position.x}, {msg.position.y}, {msg.position.z}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}")
        # print(f"self.alpha = {self.alpha}")
        # print(f"self.beta = {self.beta}")
        # print(f"self.gamma = {self.gamma}")
        # print(f"self.rho = {self.rho}")
        # print(f"self.psi = {self.psi}")
        # print("")
        
        # angles = np.stack([self.alpha, self.beta, self.gamma])
        
        self.publish_legs()
        # print(f"took: {time.time()-start}s")
        rospy.sleep(0.01)
        return

    def ik_serial(self, s, u, i):
        l = self.O + self.P + self.R.dot(s) - u
        
        alpha = np.arctan(l[1] / l[0])

        l_1 = l1_m * np.array([np.cos(alpha), np.sin(alpha), 0]) * np.array([(-1)**(i), (-1)**(i), 1])

        s_2 = s + l_1
        
        l_prime = self.O + self.P + self.R.dot(s_2) - u

        l_primem = np.linalg.norm(l_prime, 2)

        gamma = pi-np.arccos((l2_m**2 + l3_m **2 - l_primem**2) / (2 * l2_m * l3_m))

        h = l[2]

        h_prime = l_prime[2]
        # print(h)
        # print(h_prime)

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
    
    def getPhases(self, beta, stride_time):
        support1_ends = np.zeros(n_legs)
        transfer1_ends = np.zeros(n_legs)
        #one Indexing cause matlab stuffs
        for i in range(1, n_legs+1):
            print(i)
            #Calculate how many (1-beta)'s the end of the support phase is offset by
            if i%2 == 0:
                support_offset = (((i-1)-((i-1)%2))/2) + 3
            else:
                support_offset = ((i-(i%2))/2) + 1
            #Wrap the offset 
            if support_offset > 1/(1-beta):
                support_offset = support_offset%(1/(1-beta))

            support1_end = (1-(support_offset*(1-beta)))*stride_time
            transfer1_end = support1_end + ((1-beta)*stride_time)
            
            support1_ends[i-1] = support1_end
            transfer1_ends[i-1] = transfer1_end
            
        return support1_ends, transfer1_ends
    
    def getFootTraj(self, time, total_time, height, stride_len):
        phase = time/total_time
        #Lift up the foot to 2/3 the total height
        if phase < 0.2:
            rel_phase = phase/0.2
            x = -stride_len/2 * np.cos(self.dir)
            y = -stride_len/2 * np.sin(self.dir)
            z = rel_phase * (2/3) * height
        #Lift up the last 1/3 the total height and translate 1/3 the stride length
        elif phase < 0.4:
            rel_phase = (phase-0.2)/0.2
            x = (-stride_len/2 + (1-np.cos(rel_phase*pi/2))*stride_len/3) * np.cos(self.dir)
            y = (-stride_len/2 + (1-np.cos(rel_phase*pi/2))*stride_len/3) * np.sin(self.dir)
            z = (2/3)*height + (np.sin(rel_phase*pi/2)*height/3)
        #Translate 1/3 the stride len
        elif phase < 0.6:
            rel_phase = (phase-0.4)/0.2
            x = (-stride_len/2 + stride_len/3 + rel_phase*stride_len/3) * np.cos(self.dir)
            y = (-stride_len/2 + stride_len/3 + rel_phase*stride_len/3) * np.sin(self.dir)
            z = height
        #Drop 1/3 the total heigh and translate the last 1/3 the stride length
        elif phase < 0.8:
            rel_phase = (phase-0.6)/0.2
            x = (-stride_len/2 + 2*stride_len/3 + np.cos((1-rel_phase)*pi/2)*stride_len/3) * np.cos(self.dir)
            y = (-stride_len/2 + 2*stride_len/3 + np.cos((1-rel_phase)*pi/2)*stride_len/3) * np.sin(self.dir)
            z = height - (1-np.sin((1-rel_phase)*pi/2))*height/3
        #Drop the last 2/3 the total height
        else:
            rel_phase = (phase-0.8)/0.2
            x = stride_len/2 * np.cos(self.dir)
            y = stride_len/2 * np.sin(self.dir)
            z = (1-rel_phase)*(2/3)*height
        return x,y,z
    
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
                # print(joint_msg)
                joint_msg.position = [joint_msg.position[0] + joint_offsets[i][j]]
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
    pass

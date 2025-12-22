#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
import tf 
from sensor_msgs.msg import Imu
from math import pi, sin, cos
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class OdomVisualizer:
    def __init__(self):
        rospy.init_node('odom_plotter', anonymous=True)

        # Data storage
        self.gt_x, self.gt_y = [], []
        self.odom_x, self.odom_y = [], []

        self.initial_pos = Point(-5.0,-4.0,0)

        # Subscribers (Update these topic names to match your setup)
        rospy.Subscriber('/ground_truth', Point, self.gt_callback)
        rospy.Subscriber('/odom/filtered', Odometry, self.odom_callback)

        # Setup Plot
        self.fig, self.ax = plt.subplots()
        self.ln_gt, = plt.plot([], [], 'g-', label='Ground Truth')
        self.ln_odom, = plt.plot([], [], 'r--', label='Odometry')
        
        plt.legend()
        plt.title("Robot Trajectory: Ground Truth vs Odometry")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.grid(True)


    def gt_callback(self, msg:Point):
        self.gt_x.append(msg.x)
        self.gt_y.append(msg.y)

    def odom_callback(self, msg:Odometry):
        self.odom_x.append(msg.pose.pose.position.x+self.initial_pos.x)
        self.odom_y.append(msg.pose.pose.position.y+self.initial_pos.y)

    def init_plot(self):
        self.ax.set_xlim(-20, 20) # Adjust based on your environment size
        self.ax.set_ylim(-20, 20)
        return self.ln_gt, self.ln_odom

    def update_plot(self, frame):
        self.ln_gt.set_data(self.gt_x, self.gt_y)
        self.ln_odom.set_data(self.odom_x, self.odom_y)
        
        # # Optional: Auto-adjust axis limits
        # if self.gt_x and self.gt_y:
        #     self.ax.set_xlim(min(self.gt_x + self.odom_x) - 1, max(self.gt_x + self.odom_x) + 1)
        #     self.ax.set_ylim(min(self.gt_y + self.odom_y) - 1, max(self.gt_y + self.odom_y) + 1)
            
        return self.ln_gt, self.ln_odom

if __name__ == '__main__':
    viz = OdomVisualizer()
    ani = FuncAnimation(viz.fig, viz.update_plot, init_func=viz.init_plot, blit=True)
    plt.show()
    rospy.spin()
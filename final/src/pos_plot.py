#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class OdomVisualizer:
    def __init__(self):
        rospy.init_node('odom_plotter', anonymous=True)

        # Data storage
        self.gt_x, self.gt_y = [], []
        self.odom_x, self.odom_y = [], []
        
        # RMSE storage
        self.sq_errors = []
        self.current_rmse = 0.0

        self.finished = False
        self.initial_pos = Point(-4.0, -4.0, 0)

        # Subscribers
        rospy.Subscriber('/ground_truth', Point, self.gt_callback)
        rospy.Subscriber('/odom/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/replay_finished', String, self.finish_callback)

        # Setup Plot
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ln_gt, = self.ax.plot([], [], 'g-', label='Ground Truth', linewidth=1.5)
        self.ln_odom, = self.ax.plot([], [], 'r--', label='Odometry', linewidth=1.5)
        
        # Text box for RMSE
        self.rmse_text = self.ax.text(0.05, 0.95, '', transform=self.ax.transAxes, 
                                     fontsize=12, fontweight='bold', verticalalignment='top',
                                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

        self.ax.set_aspect('equal') # Ensures 1m X = 1m Y visually
        self.ax.legend(loc='lower right')
        self.ax.set_title("Ground Truth vs Odometry")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.grid(True)

    def gt_callback(self, msg: Point):
        self.gt_x.append(msg.x)
        self.gt_y.append(msg.y)

    def odom_callback(self, msg: Odometry):
        # Calculate current position with offset
        curr_ox = msg.pose.pose.position.x + self.initial_pos.x
        curr_oy = msg.pose.pose.position.y + self.initial_pos.y
        
        self.odom_x.append(curr_ox)
        self.odom_y.append(curr_oy)

        # Calculate RMSE using the most recent Ground Truth point
        if self.gt_x and self.gt_y:
            latest_gx = self.gt_x[-1]
            latest_gy = self.gt_y[-1]
            
            # Distance squared between Odom and GT
            dist_sq = (latest_gx - curr_ox)**2 + (latest_gy - curr_oy)**2
            self.sq_errors.append(dist_sq)
            
            # Update RMSE: sqrt(mean(errors^2))
            self.current_rmse = np.sqrt(np.mean(self.sq_errors))

    def finish_callback(self, msg: String):
        rospy.loginfo("Received finish signal.")
        self.finished = True


    def init_plot(self):
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        return self.ln_gt, self.ln_odom, self.rmse_text

    def update_plot(self, frame):
        if not self.finished:
            # Update line data
            self.ln_gt.set_data(self.gt_x, self.gt_y)
            self.ln_odom.set_data(self.odom_x, self.odom_y)
            
            # Update RMSE text display
            self.rmse_text.set_text(f"RMSE: {self.current_rmse:.4f} m")
            
            # Auto-adjust limits if data exists
            if self.gt_x and self.odom_x:
                all_x = self.gt_x + self.odom_x
                all_y = self.gt_y + self.odom_y
                self.ax.set_xlim(min(all_x) - 2, max(all_x) + 2)
                self.ax.set_ylim(min(all_y) - 2, max(all_y) + 2)
            
        return self.ln_gt, self.ln_odom, self.rmse_text

if __name__ == '__main__':
    try:
        viz = OdomVisualizer()
        # Interval set to 100ms (10Hz refresh rate)
        ani = FuncAnimation(viz.fig, viz.update_plot, init_func=viz.init_plot, 
                            blit=True, interval=100)
        plt.show()
    except rospy.ROSInterruptException:
        pass
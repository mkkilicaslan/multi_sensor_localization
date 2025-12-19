#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math


class WheelOdom:
    def __init__(self):
        rospy.init_node('joint_state_odom_node')

        # --- Physical Parameters ---
        self.wheel_radius = 0.098  # meters
        self.wheel_separation = 0.37559  # meters

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.th = 0.0
        self.vth = 0.0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.rate = rospy.Rate(10)  # Hz

    
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        self.odom_pub = rospy.Publisher("/odom/wheel", Odometry, queue_size=10)

        self.run()

    def joint_callback(self, data):
        self.current_time = rospy.Time.now()

        [front_left, front_right, back_left, back_right] = [vel for vel in data.velocity]

        v_left_rad = (front_left + back_left) / 2.0
        v_right_rad = (front_right + back_right) / 2.0

        # 3. Convert rad/s to linear velocity (m/s)
        v_left = v_left_rad * self.wheel_radius
        v_right = v_right_rad * self.wheel_radius

        # 4. Differential Kinematics
        self.vx = (v_left + v_right) / 2.0
        self.vth = (v_right - v_left) / self.wheel_separation

        # 5. Integrate for Pose
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th


        self.last_time = self.current_time


    def run(self):
        while not rospy.is_shutdown():
            quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position = Point(self.x, self.y, 0.)
            odom.pose.pose.orientation = Quaternion(*quat)
            odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

            self.odom_pub.publish(odom)


            self.rate.sleep()



if __name__ == '__main__':
    WheelOdom()

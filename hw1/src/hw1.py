#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32

class Class():
    def __init__(self):
        rospy.init_node("node_name")

        self.rate = rospy.Rate(10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.x_pub = rospy.Publisher('/x_position', Float32, queue_size=10)
        self.y_pub = rospy.Publisher('/y_position', Float32, queue_size=10)

        self.twist = Twist()
        self.twist.linear.x = 1
        self.twist.angular.z = 0.5
        self.pos = Point()

        self.run()


    def odom_cb(self, msg:Odometry):
        self.pos = msg.pose.pose.position

        self.x_pub.publish(Float32(self.pos.x))
        self.y_pub.publish(Float32(self.pos.y))


    def run(self):
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)

            self.rate.sleep()



if __name__ == '__main__':
    Class()

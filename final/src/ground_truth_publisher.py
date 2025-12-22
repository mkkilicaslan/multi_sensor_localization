#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
import tf 
from sensor_msgs.msg import Imu
from math import pi, sin, cos
from gazebo_msgs.msg import ModelStates


class GroundTruth():
    def __init__(self):
        rospy.init_node("ground_truth_publisher")

        self.rate = rospy.Rate(10)

        self.pos = Point(0.0, 0.0, 0.0)
        self.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_cb)

        self.pub = rospy.Publisher("/ground_truth", Point, queue_size=10)
        self.run()



    def model_cb(self, msg: ModelStates):
        idx = msg.name.index("jackal")
        self.pos = msg.pose[idx].position
        self.twist = msg.twist[idx]



    def run(self):
        while not rospy.is_shutdown():
            # rospy.loginfo(f"Position: x={self.pos.x}, y={self.pos.y}, z={self.pos.z}")
            self.pub.publish(self.pos)
    
            self.rate.sleep()



if __name__ == '__main__':
    GroundTruth()

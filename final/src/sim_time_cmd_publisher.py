#!/usr/bin/env python3

import rospy
import rosbag
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def replay_at_fixed_freq(bag_path, frequency):
    rospy.init_node('fixed_freq_replayer')
    
    # Ensure we are synced with Gazebo
    if not rospy.get_param('/use_sim_time', False):
        rospy.logwarn("use_sim_time is FALSE. Commands will follow system clock, not Gazebo.")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    finished_pub = rospy.Publisher('/replay_finished', String, queue_size=1)

    rate = rospy.Rate(frequency) # This follows Sim Time if use_sim_time is true
    
    rospy.loginfo(f"Opening bag: {bag_path} at {frequency}Hz")
    
    try:
        bag = rosbag.Bag(bag_path)
        # Generator to fetch messages one by one
        msg_generator = bag.read_messages(topics=['cmd_vel'])

        for topic, msg, t in msg_generator:
            if rospy.is_shutdown():
                break
            
            # Publish the message
            pub.publish(msg)
            

            rospy.loginfo("Playing.")
            # Sleep to maintain the desired frequency
            # If Gazebo is slow, this sleep will automatically stretch
            rate.sleep()
            
        bag.close()
        rospy.logwarn("Replay finished.")
        finished_pub.publish(String(data="finished"))


    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    BAG_FILE = '/home/mami/robot_autonomy/src/command.bag'
    REPLAY_HZ = 60 
    
    replay_at_fixed_freq(BAG_FILE, REPLAY_HZ)
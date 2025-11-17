#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomToPoseConverter:
    def __init__(self):
        rospy.init_node('odom_to_pose_converter')
        
        # Get parameters
        self.input_topic = rospy.get_param('~input_topic', '/odom')
        self.output_topic = rospy.get_param('~output_topic', '/odom')
        
        # Subscriber and publisher
        self.subscriber = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback)
        self.publisher = rospy.Publisher(self.output_topic, PoseWithCovarianceStamped, queue_size=10)
        
        rospy.loginfo("OdomToPoseConverter: Subscribing to %s, publishing to %s", self.input_topic, self.output_topic)
    
    def odom_callback(self, msg):
        """Convert nav_msgs::Odometry to geometry_msgs::PoseWithCovarianceStamped"""
        pose_msg = PoseWithCovarianceStamped()
        
        # Copy header
        pose_msg.header = msg.header
        
        # Copy pose and covariance
        pose_msg.pose = msg.pose
        
        # Publish
        self.publisher.publish(pose_msg)

if __name__ == '__main__':
    try:
        converter = OdomToPoseConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/python
PKG = 'misc_bag_utils'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import cv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import pdb
import matplotlib.pyplot as plot
from calculate_time_offset import OffsetEstimator
import os
import sys

 
class BagAligner():
    
    def __init__(self):
        self.input_bag_name = rospy.get_param('~input_file_name')
        self.output_bag_name = rospy.get_param('~output_file_name', default=os.path.splitext(self.input_bag_name)[0]+'_aligned.bag')
        
        self.imu_topic = rospy.get_param('~imu_topic')

        self.align_odom = rospy.get_param('~align_odom')
        if self.align_odom:
            self.num_odom = rospy.get_param('~num_odom')
            self.odom_topics = []
            for i in range(self.num_odom):
                self.odom_topics.append(rospy.get_param('~odom_topic'+str(i+1)))
        
        self.align_camera = rospy.get_param('~align_camera')
        if self.align_camera:
            self.left_camera_topic = rospy.get_param('~left_camera_topic')
            self.right_camera_topic = rospy.get_param('~right_camera_topic')
            self.camera_dt = rospy.get_param('imu_camera_dt')

        self.odom_data = [[] for i in range(len(self.odom_topics))]
        self.imu_data = []
        
    
    def read_bag(self):
        #Get a list of all the topics in the rosbag
        rospy.loginfo("[BagAligner] Reading bag" + self.input_bag_name)
        with rosbag.Bag(self.input_bag_name, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == self.imu_topic:
                    self.imu_data.append([t.to_sec(), np.array([msg.angular_velocity.x,msg.angular_velocity.y, msg.angular_velocity.z])])
                    continue

                if self.align_odom:
                    index = [i for i, val in enumerate(self.odom_topics) if val == topic]
                    if index:
                        self.odom_data[index[0]].append([t.to_sec(), np.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z])]);

        rospy.loginfo("[BagAligner] ...Done!")

    def write_bag(self):

        time_shifts = []
        if self.align_odom:
            for i, topic_name in enumerate(self.odom_topics):
                EstimatorObject = OffsetEstimator(topic_name,  self.imu_topic)
                EstimatorObject.odom_data = self.odom_data[i]
                EstimatorObject.imu_data = self.imu_data

                time_shifts.append(EstimatorObject.estimate_offset())

        if not time_shifts and not align_camera:
            rospy.logwarn('No topics provided, nothing to write')
            return

        rospy.loginfo("[BagAligner] Writing Aligned bag to "+self.output_bag_name)
        with rosbag.Bag(self.output_bag_name, 'w') as out_bag:
            with rosbag.Bag(self.input_bag_name, 'r') as in_bag:
                for topic, msg, t in in_bag.read_messages():
                    msg_t = t
                    
                    if self.align_odom:
                        index = [i for i, val in enumerate(self.odom_topics) if val == topic]
                        if index:
                            msg.header.stamp = msg.header.stamp - rospy.Duration(time_shifts[index[0]])
                            msg_t = msg_t - rospy.Duration(time_shifts[index[0]])

                    if self.align_camera:
                        if topic == self.left_camera_topic or topic == self.right_camera_topic:
                            msg.header.stamp = msg.header.stamp - rospy.Duration(camera_dt)
                            msg_t = msg_t - rospy.Duration(camera_dt)

                    out_bag.write(topic, msg, msg_t);

        rospy.loginfo("[BagAligner] ...Done!!!")

                    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('bag_aligner')
    try:
        AlignObject = BagAligner()
        AlignObject.read_bag()
        AlignObject.write_bag()
        rospy.loginfo('[BagAligner] Fin.')
    except rospy.ROSInterruptException: 
        pass
    rospy.spin()
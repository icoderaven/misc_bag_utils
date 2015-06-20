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

# Reading bag input_file_name from command line or roslaunch parameter.
import os
import sys

 
class OffsetEstimator():
    
    def __init__(self):
        self.odom_data = []
        self.imu_data = []
        
    def __init__(self, odom_topic, imu_topic):
        self.odom_data = []
        self.imu_data = []
        self.odom_topic = odom_topic
        self.imu_topic = imu_topic
    
    def read_bag(self, file_name):
        #Get a list of all the topics in the rosbag
        rospy.loginfo("[OffsetEstimator] Reading bag" + input_file_name)
        with rosbag.Bag(input_file_name, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == self.odom_topic:
                    self.odom_data.append([t.to_sec(), np.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z])]);

                if topic == self.imu_topic:
                    self.imu_data.append([t.to_sec(), np.array([msg.angular_velocity.x,msg.angular_velocity.y, msg.angular_velocity.z])])

        rospy.loginfo("[OffsetEstimator] ...Done!")

    def plot_data(self, odom, imu):
        self.figure = plot.figure(0, figsize=(12,12))
        self.axes = self.figure.add_subplot(111)
        self.axes.cla()
        self.axes.plot(odom, 'r.', label=self.odom_topic,markersize=1)
        self.axes.plot(imu, 'bx', label=self.imu_topic,markersize=1)
        self.axes.set_xlabel('Index')
        self.axes.set_ylabel('Norm angular velocity (rad/s)')
        # self.axes.set_xbound(-bound,bound)
        # self.axes.set_ybound(-bound,bound)
        self.axes.grid(True, axis='both')
        self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,ncol=2, mode="expand", borderaxespad=0., numpoints=5)           
        plot.show()
        plot.close()


    def estimate_offset(self):
        if not self.odom_data or not self.imu_data:
            rospy.logfatal('[OffsetEstimator] Did not read data topics!')
            exit(-1)
        #Find the correlation between the norms of the two velocities, and then find the index of the peak of the correlation to find the offset index
        odom_norm = []
        imu_norm = []
        imu_t = []
        odom_t = []

        imu_norm = [np.linalg.norm(imu[1]) for imu in self.imu_data]
        imu_t = [imu[0] for imu in self.imu_data]
        odom_norm = [np.linalg.norm(odom[1]) for odom in self.odom_data]    
        odom_t = [odom[0] for odom in self.odom_data]

        #Regularize
        odom_norm = np.array(odom_norm)
        imu_norm = np.array(imu_norm)
        imu_t = np.array(imu_t);
        odom_t = np.array(odom_t)

        odom_norm -= odom_norm.mean()
        odom_norm /= odom_norm.std()

        imu_norm -= imu_norm.mean()
        imu_norm /= imu_norm.std()

        #Now the rates of the two streams might be different; thus we need to interpolate. Choose to interpolate odom to IMU
        #@todo: The larger should be interpolated to the smaller of the two
        dT = np.mean(np.diff(imu_t))
        dT2 = np.mean(np.diff(odom_t))
        rospy.loginfo('[OffsetEstimator] imu rate :: ' + str(1.0/ dT) + 'Hz, mocap rate :: ' + str(1.0 /dT2) +' Hz')

        rate_adjusted_odom_norm = np.interp(imu_t, odom_t, odom_norm);
        
        corr = np.correlate(rate_adjusted_odom_norm, imu_norm, "full")
        index_shift = np.argmax(corr) - (np.size(imu_norm)-1)
        

        time_shift = self.imu_data[len(self.imu_data)/2 + index_shift][0] - self.imu_data[len(self.imu_data)/2][0];
        # time_shift = dT * index_shift
        
        self.plot_data(rate_adjusted_odom_norm, imu_norm)

        rospy.loginfo('[OffsetEstimator] Time shift (imu_t + t_d) found to be ' + str(time_shift) + ' sec')
        return time_shift

    def write_bag(self, file_name, output_file_name):
        time_shift = self.estimate_offset()
        rospy.loginfo("[OffsetEstimator] Writing Aligned bag to "+output_file_name)
        with rosbag.Bag(output_file_name, 'w') as out_bag:
            with rosbag.Bag(input_file_name, 'r') as in_bag:
                for topic, msg, t in in_bag.read_messages():
                    msg_t = t
                    if topic == self.odom_topic:
                        msg.header.stamp = msg.header.stamp - rospy.Duration(time_shift)
                        msg_t = msg_t - rospy.Duration(time_shift)

                    out_bag.write(topic, msg, msg_t);

        rospy.loginfo("[OffsetEstimator] ...Done!!!")

                    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('estimate_temporal_offset')
    input_file_name = rospy.get_param('~input_file_name')
    output_file_name = rospy.get_param('~output_file_name', default=os.path.splitext(input_file_name)[0]+'_aligned.bag')

    odom_topic = rospy.get_param('~odom_topic')
    imu_topic = rospy.get_param('~imu_topic')

    try:
        EstimatorObject = OffsetEstimator(odom_topic, imu_topic)
        EstimatorObject.read_bag(input_file_name)
        # EstimatorObject.estimate_offset()
        EstimatorObject.write_bag(input_file_name, output_file_name)
        rospy.loginfo('[OffsetEstimator] Fin.')
    except rospy.ROSInterruptException: 
        pass
    rospy.spin()
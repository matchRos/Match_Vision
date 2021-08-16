#!/usr/bin/python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
import tf
import numpy as np
import math


pose = PoseWithCovarianceStamped()
encoder_odom = Odometry()

def enc_odom_cb(data):
    global encoder_odom
    encoder_odom = data
def pose2odom_cb(data):
    global pose
    pose = data

rospy.init_node('pose_to_odom_combined')

pose2odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, pose2odom_cb, queue_size=100)
encode_sub = rospy.Subscriber('/odom', Odometry, enc_odom_cb, queue_size=100)


odom_pub = rospy.Publisher('/odom_combined', Odometry, queue_size=100)

rate = rospy.Rate(50.0)
counter = 0
x = 0.
y = 0.
dt = 1./50.

while not rospy.is_shutdown():

    if counter > 0:
        
        odom = Odometry()
        encoder_odom = Odometry()
        odom.header.frame_id = 'odom_combined'
        odom.child_frame_id = 'base_footprint'
        odom.header.stamp = pose.header.stamp

        odom.pose.pose.position.x = pose.pose.pose.position.x
        odom.pose.pose.position.y = pose.pose.pose.position.y
        odom.pose.pose.position.z = pose.pose.pose.position.z

        odom.pose.pose.orientation.x = pose.pose.pose.orientation.x
        odom.pose.pose.orientation.y = pose.pose.pose.orientation.y
        odom.pose.pose.orientation.z = pose.pose.pose.orientation.z
        odom.pose.pose.orientation.w = pose.pose.pose.orientation.w
        
        odom.pose.covariance = encoder_odom.pose.covariance
        odom.twist.twist.linear.x = encoder_odom.twist.twist.linear.x
        odom.twist.twist.linear.y = 0
        odom.twist.twist.linear.z = 0

        odom.twist.twist.angular.x = 0.
        odom.twist.twist.angular.y = 0.
        odom.twist.twist.angular.z = encoder_odom.twist.twist.angular.z
        
        odom.twist.covariance = encoder_odom.twist.covariance
        odom_pub.publish(odom)
        
        br = tf.TransformBroadcaster()
        if not pose.pose.pose.orientation.x==0 and pose.pose.pose.orientation.y==0 and pose.pose.pose.orientation.z ==0 and pose.pose.pose.orientation.w ==0:
            br.sendTransform((x,y,z),[pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w],pose.header.stamp, "base_footprint","odom_combined")



    else:
        x_prev = x
        y_prev = y

        counter += 1

    

    rate.sleep()


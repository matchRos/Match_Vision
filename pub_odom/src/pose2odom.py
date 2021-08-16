#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
import tf
import numpy as np
import math


pose = PoseWithCovarianceStamped()
encoder_odom = Odometry()
orb_pose = PoseStamped()
def enc_odom_cb(data):
    global encoder_odom
    encoder_odom = data
def pose2odom_cb(data):
    global pose
    pose = data
def orb_odom_cb(data):
    global orb_pose
    orb_pose = data
T_cam_imu = [[1.0, 0.0, 0.0, 0.0],
[0.0, 1.0, 0.0, 0.0],
[0.0, 0.0, 1, 0.0],
[0.0, 0.0, 0.0, 1.0]]
def euler_angle_trans(a, b, c, x, y, z, T):
    # This function is the transformation in the form of Euler angles and coordinates
    # input: a,b,c: Euler angles before transformation
    #        x,y,z: coordiantes before transformation
    #        T: transformation matrix
    # output: a_2, b_2, c_2: Euler angles after transformation
    #         x_2, y_2, z_2: coordiantes before transformation
    theta = np.zeros((3, 1), dtype=np.float64)
    theta[0] = a*3.141592653589793/180.0
    theta[1] = b*3.141592653589793/180.0
    theta[2] = c*3.141592653589793/180.0
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    T_1 = np.empty((4,4))
    T_1[:3, :3] = R
    T_1[:3, 3] = np.array([x, y, z])
    T_1[3, :] = [0, 0, 0, 1]
    T_2 = np.dot(T, T_1)
    R_2 = np.empty((3, 3))
    x_2 = T_2[0, 3]
    y_2 = T_2[1, 3]
    z_2 = T_2[2, 3]
    R_2 = T_2[:3, :3]
    sy = math.sqrt(R_2[0,0] * R_2[0,0] +  R_2[1,0] * R_2[1,0])
    singular = sy < 1e-6
    if  not singular:
        a_2 = math.atan2(R_2[2,1] , R_2[2,2])
        b_2 = math.atan2(-R_2[2,0], sy)
        c_2 = math.atan2(R_2[1,0], R_2[0,0])
    else :
        a_2 = math.atan2(-R_2[1,2], R_2[1,1])
        b_2 = math.atan2(-R_2[2,0], sy)
        c_2 = 0

    a_2 = a_2*180.0/3.141592653589793
    b_2 = b_2*180.0/3.141592653589793
    c_2 = c_2*180.0/3.141592653589793
    return a_2, b_2, c_2, x_2, y_2, z_2
def quaternion_to_euler_angle(w, x, y, z):
    # This function is to convert Quaternion to Euler angles
    # input: w,x,y,z: Quaternion
    # output: X, Y, Z: Euler angles
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z
rospy.init_node('pose_to_odom')

pose2odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, pose2odom_cb, queue_size=100)
encode_sub = rospy.Subscriber('/odom', Odometry, enc_odom_cb, queue_size=100)
vlslam_sub = rospy.Subscriber('/orb_pose', PoseStamped, orb_odom_cb, queue_size=100)

odom_pub = rospy.Publisher('/odom_combined', Odometry, queue_size=100)
vslam_pub = rospy.Publisher('/odom_orb', Odometry, queue_size=100)
rate = rospy.Rate(22.0)
counter = 0
x = 0.
y = 0.
dt = 1./22.

while not rospy.is_shutdown():

    if counter > 0:
        (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(orb_pose.pose.orientation.w, orb_pose.pose.orientation.x , orb_pose.pose.orientation.y, orb_pose.pose.orientation.z)
        v_phi_0 = float((v_roll))
        v_theta_0 = float((v_pitch))
        v_psi_0 = float((v_yaw))
    
        x_0 = orb_pose.pose.position.x
        y_0 = orb_pose.pose.position.y
        z_0 = orb_pose.pose.position.z

        (v_phi,v_theta,v_psi,x,y,z) = euler_angle_trans(v_phi_0,v_theta_0,v_psi_0,x_0,y_0,z_0,T_cam_imu)

        yaw = math.radians(v_psi)
        vel_x_world = (x - x_prev) / dt
        vel_y_world = (y - y_prev) / dt


        x_prev = x
        y_prev = y
        

        twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world

        orb_odom = Odometry()
        orb_odom.header.frame_id = 'odom_orb'
        orb_odom.child_frame_id = 'base_footprint'
        orb_odom.header.stamp = orb_pose.header.stamp

        orb_odom.pose.pose.position.x = orb_pose.pose.position.x
        orb_odom.pose.pose.position.y = orb_pose.pose.position.y
        orb_odom.pose.pose.position.z = orb_pose.pose.position.z

        orb_odom.pose.pose.orientation.x = orb_pose.pose.orientation.x
        orb_odom.pose.pose.orientation.y = orb_pose.pose.orientation.y
        orb_odom.pose.pose.orientation.z = orb_pose.pose.orientation.z
        orb_odom.pose.pose.orientation.w = math.sqrt(1-pow(orb_pose.pose.orientation.x,2)-pow(orb_pose.pose.orientation.y,2)-pow(orb_pose.pose.orientation.z,2))

        orb_odom.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e5]
    
        orb_odom.twist.twist.linear.x = twist_x
        orb_odom.twist.twist.linear.y = 0
        orb_odom.twist.twist.linear.z = 0 
        orb_odom.twist.twist.angular.x = 0.
        orb_odom.twist.twist.angular.y = 0.
        orb_odom.twist.twist.angular.z = 0.
        orb_odom.twist.covariance = [1e3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e5]
        #if not orb_pose.pose.position.x==0 and orb_pose.pose.position.y ==0 and orb_pose.pose.position.z ==0 and orb_pose.pose.orientation.x ==0 and orb_pose.pose.orientation.y ==0 and orb_pose.pose.orientation.z ==0 :
        vslam_pub.publish(orb_odom)
        #    print(orb_odom.pose.pose.position.x)
        #    vslam_pub.publish(orb_odom)
        


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
        
        odom.pose.covariance = pose.pose.covariance
        odom.twist.twist.linear.x = encoder_odom.twist.twist.linear.x
        odom.twist.twist.linear.y = 0.
        odom.twist.twist.linear.z = 0.

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

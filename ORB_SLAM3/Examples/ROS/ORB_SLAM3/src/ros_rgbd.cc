/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include "geometry_msgs/TransformStamped.h"

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<opencv2/core/core.hpp>

#include <geometry_msgs/PoseStamped.h> 
#include <tf/tf.h> 
#include"../../../include/Converter.h"
#include <cmath>
#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_rect_color", 30);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 30);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("orb_pose", 100);
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_rgbd.txt");

    SLAM.SaveTrajectoryTUM("FrameTrajectory_rgbd.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    //  cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    // if (pose.empty())
    //     return;

    // /* global left handed coordinate system */
    //static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    // static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
    //                                                            -1, 1,-1, 1,
    //                                                            -1,-1, 1, 1,
    //                                                             1, 1, 1, 1);

    // //prev_pose * T = pose
    // cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    // world_lh = world_lh * translation;
    // pose_prev = pose.clone();


    /* transform into global right handed coordinate system, publish in ROS*/
    tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                   - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                     world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    // //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
     const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                             0, 0, 1,
                                             1, 0, 0);

    // static tf::TransformBroadcaster br;

     tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
     tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
     tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "orb_odom"));


// This part is to output pose of camera as rosmsg
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("orb_pose", 100);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cv_ptrRGB->header.stamp;//ros::Time::now();
    pose.header.frame_id ="map";
    if(Tcw.empty()){
        cout<<"Track lost!"<<endl;
    }
    else{
    // float temp = pose.pose.position.x;
    // pose.pose.position.x = pose.pose.position.z;
    // pose.pose.position.z = pose.pose.position.y;
    // pose.pose.position.y = -temp;
    // pose.pose.orientation.z = pose.pose.orientation.y;
    // pose.pose.orientation.y = 0;
    //cv::Mat rotation270deg  = (cv::Mat_<float>(3,3)<<1,0,0,0,1,0,0,0,1);
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc); // Quaternion

    tf::Transform new_transform;
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

    tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
    new_transform.setRotation(quaternion);
    //tf::Transform transform2 = new_transform*transform;
    tf::poseTFToMsg(new_transform, pose.pose);
    float temp = pose.pose.position.x;
    pose.pose.position.x = pose.pose.position.z;
    pose.pose.position.z = 0;
    pose.pose.position.y = -temp;
    pose.pose.orientation.z = -pose.pose.orientation.y;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.w = sqrt(1-pow(pose.pose.orientation.x,2)-pow(pose.pose.orientation.y,2)-pow(pose.pose.orientation.z,2));
    pose_pub.publish(pose);
    
    }
}


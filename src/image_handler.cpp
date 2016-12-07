/*
* image_handler.h
* Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
* Created on: 10/01/2014
* Author: Jéremie Deray
*/

#include "ros_imresize/image_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>
#include <opencv2/imgproc/imgproc.hpp>


////////////////////////////////////////////////////////////////////////////
////////////                                                    ////////////
////////////                   SingleImageHandler               ////////////
////////////                                                    ////////////
////////////////////////////////////////////////////////////////////////////



SingleImageHandler::SingleImageHandler() :
_infoReceived(false),
_nh("~"),
_width(640),
_height(480),
_it(_nh)
{
    std::string imgTopicName ("/stereo/left/image_raw");
    std::string infoTopicName ("/stereo/left/camera_info");
    std::string newTopicName("/stereo_cam/left");
    bool _undistort = false;
    ros::Rate wrait(10);


    _nh.getParam("resize_width", _width);
    _nh.getParam("resize_height", _height);
    _nh.getParam("topic_crop", imgTopicName);
    _nh.getParam("camera_info", infoTopicName);
    _nh.getParam("namespaceTopics", newTopicName);
    _nh.getParam("undistord", _undistort);

std::cout<<"***************************************************************"<<imgTopicName<<std::endl;
std::cout<<"***************************************************************"<<newTopicName<<std::endl;

    ros::Subscriber sub_info = _nh.subscribe(infoTopicName, 1, &SingleImageHandler::setCameraInfo, this);

    _sub_img = _it.subscribe(imgTopicName, 1, &SingleImageHandler::topicCallback, this);

    _pub_img = _it.advertise(newTopicName + std::string("/image_raw"), 1);

    _pub_info = _nh.advertise<sensor_msgs::CameraInfo>(newTopicName + std::string("/camera_info"), 1);

    ROS_INFO("Running\n");

    while(ros::ok())
    {
	ros::spinOnce();
	wrait.sleep();
    }
}

SingleImageHandler::~SingleImageHandler()
{
}

void SingleImageHandler::topicCallback(const sensor_msgs::ImageConstPtr& received_image)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::BGR8);
       
    cv::Mat undist;
 
    if (_undistord)
    {
       cv::undistort(cvPtr->image, undist, _K, _dist);
    }
    else
    {
       undist = cvPtr->image;
    }   

    cv::resize(undist, cvPtr->image, cv::Size(_width, _height),
               0, 0, cv::INTER_LINEAR);

    _pub_img.publish(cvPtr->toImageMsg());
    _pub_info.publish(_infoCam);
}

void SingleImageHandler::setCameraInfo(const sensor_msgs::CameraInfoConstPtr &received_info)
{
    _infoCam = *received_info;

    float scale_x = (float)(_width) / (float)(_infoCam.width);
    float scale_y = (float)(_height) / (float)(_infoCam.height);

    _infoCam.K[0] *= scale_x;
    _infoCam.K[2] *= scale_x;

    _infoCam.K[4] *= scale_y;
    _infoCam.K[5] *= scale_y;

    //ROS_INFO_STREAM("Previous camera info :\n" << *received_info << "\n");

    if (_undistord)
    {
        _K = cv::Mat::eye(3, 3, CV_32F);

        _K.at<float>(0) = _infoCam.K[0];
        _K.at<float>(2) = _infoCam.K[2];

        _K.at<float>(4) = _infoCam.K[4];
        _K.at<float>(5) = _infoCam.K[5];

        if (_infoCam.distortion_model == "plumb_bob")
        {
            _dist = cv::Mat(_infoCam.D);
        }
        else
        {
            _dist = cv::Mat::zeros(5, 1, CV_32F);
            //TODO : check for other model
        }

        _infoCam.distortion_model = "";
        _infoCam.D.clear();

        _infoCam.D.clear();

        //ROS_INFO_STREAM("Undistortion active with param :\n" << _dist << "\n");
        //ROS_INFO_STREAM("Undistortion active with param :\n" << _K << "\n");
    }

    _infoCam.width = _width;
    _infoCam.height = _height;

    //ROS_INFO_STREAM("New camera info :\n" << _infoCam << "\n");

    _infoReceived = true;

}

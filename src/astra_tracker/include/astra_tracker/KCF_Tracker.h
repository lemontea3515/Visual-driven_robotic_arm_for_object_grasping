//
// Created by yahboom on 2021/7/30.
//

#ifndef TRANSBOT_ASTRA_KCF_TRACKER_H
#define TRANSBOT_ASTRA_KCF_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kcftracker.h"
#include <dynamic_reconfigure/server.h>
#include <time.h>

using namespace std;
using namespace cv;

class ImageConverter {
public:
    ros::NodeHandle n;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    const char *RGB_WINDOW = "rgb_img";
    const char *DEPTH_WINDOW = "depth_img";
    bool enable_get_depth = false;
    float dist_val[5];
    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool LAB = false;
    KCFTracker tracker;

    ImageConverter(ros::NodeHandle &n);

    ~ImageConverter();

    void Reset();

    void Cancel();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    void depthCb(const sensor_msgs::ImageConstPtr &msg);

};


#endif //TRANSBOT_ASTRA_KCF_TRACKER_H

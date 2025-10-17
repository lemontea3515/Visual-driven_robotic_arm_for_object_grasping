//
// Created by yahboom on 2021/7/30.
//

#include "KCF_Tracker.h"

Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;
float minDist = 0;
const int &ACTION_ESC = 27;

void onMouse(int event, int x, int y, int, void *) {
    if (select_flag) {
        selectRect.x = MIN(origin.x, x);
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);
        selectRect.height = abs(y - origin.y);
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == 1) {
//    if (event == CV_EVENT_LBUTTONDOWN) {
        bBeginKCF = false;
        select_flag = true;
        minDist = 0;
        origin = Point(x, y);
        selectRect = Rect(x, y, 0, 0);
    } else if (event == 4) {
//    } else if (event == CV_EVENT_LBUTTONUP) {
        select_flag = false;
        bRenewROI = true;
    }
}

ImageConverter::ImageConverter(ros::NodeHandle &n) {
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = n.subscribe("/ascamera_hp60c/rgb0/image", 1, &ImageConverter::imageCb, this);
    depth_sub_ = n.subscribe("/ascamera_hp60c/depth0/image_raw", 1, &ImageConverter::depthCb, this);
    namedWindow(RGB_WINDOW);
//        namedWindow(DEPTH_WINDOW);
}

ImageConverter::~ImageConverter() {
    n.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    destroyWindow(RGB_WINDOW);
//        destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    enable_get_depth = false;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // 使用 BGR8 格式
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(rgbimage);
    setMouseCallback(RGB_WINDOW, onMouse, 0);
    if (bRenewROI) {
        if (selectRect.width <= 0 || selectRect.height <= 0) {
            bRenewROI = false;
            return;
        }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if (bBeginKCF) {
        enable_get_depth = true;
        result = tracker.update(rgbimage);
        
        // 计算 center_x 和 center_y
        int center_x = result.x + result.width / 2;
        int center_y = result.y + result.height / 2;
        
        // 绘制矩形框
        rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
        
        // 绘制中心点
        circle(rgbimage, Point(center_x, center_y), 3, Scalar(0, 0, 255), -1);
        
        // 更新文本信息
        string text_x = "X: " + to_string(center_x);
        string text_y = "Y: " + to_string(center_y);
        string text_z = "Z: ";
        if ((double)minDist < 0.0) text_z.append("inf");
        else text_z.append(to_string((double)(minDist)));
        text_z.append("m");

        // 在图像上绘制文本
        putText(rgbimage, text_x.c_str(), Point(center_x + 10, center_y - 13), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 0, 255), 1, 8);
        putText(rgbimage, text_y.c_str(), Point(center_x + 10, center_y + 5), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 255, 0), 1, 8);
        putText(rgbimage, text_z.c_str(), Point(center_x + 10, center_y + 23), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(255, 0, 0), 1, 8);
    } else {
        rectangle(rgbimage, selectRect, Scalar(0, 0, 255), 2, 8, 0);
    }

    imshow(RGB_WINDOW, rgbimage);

    int action = waitKey(1) & 0xFF;
    if (action == 'q' || action == ACTION_ESC) this->Cancel();
    else if (action == 'r') this->Reset();
}

void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // 这里应该是深度图像的类型，通常为 TYPE_32FC1
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);  // 确保这里使用正确的编码类型
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
    if (enable_get_depth) {
        int center_x = (int) (result.x + result.width / 2);
        int center_y = (int) (result.y + result.height / 2);
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5) / 1000;
        //printf("%f \n",dist_val[i]);
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5) / 1000;
      
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5) / 1000;
      
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5) / 1000;
       
        dist_val[4] = depthimage.at<float>(center_y, center_x) / 1000;
        
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
            
            if (dist_val[i] > 0.2 && dist_val[i] < 4.0) minDist += dist_val[i];
            else num_depth_points--;
        }
        minDist /= num_depth_points;
    }
    // imshow(DEPTH_WINDOW, depthimage);
    waitKey(1);
}


void ImageConverter::Cancel() {
    this->Reset();
    ros::Duration(0.5).sleep();  // 确保有时间退出
    cv::destroyAllWindows();  // 销毁所有 OpenCV 窗口
    ros::shutdown();  // 正常关闭 ROS 节点
}



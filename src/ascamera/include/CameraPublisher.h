/**
 * @file      CameraPublisher.h
 * @brief     angstrong camera publisher header.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */
#pragma once

#include <thread>
#include <map>
#include <unordered_map>
#include <chrono>
#include <memory>

#include "ascamera_node.h"
#include "CameraSrv.h"

#include "Logger.h"
#include "as_camera_sdk_api.h"
#include "common.h"
#include "Camera.h"

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/distortion_models.h>
// #include "depth_image_proc/depth_conversions.h"
#include "sensor_msgs/LaserScan.h"

// #include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CompressedImage.h"
//#include "sensor_msgs/sensor_fusion.pb.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

typedef enum CAMERA_STATUS_E {
    CAMERA_CLOSED_STATUS = 0,
    CAMERA_OPENED_STATUS = 1,
    CAMERA_STREAM_STATUS = 2,
    CAMERA_STREAM_BUUT,
} AS_APP_CAMERA_STATUS_E;

typedef struct PUBLISHER_INFO {
    ros::Publisher depth_camera_info_pub;
    ros::Publisher depth_raw_pub;
    ros::Publisher depth_points_pub;

    ros::Publisher ir_camera_info_pub;
    ros::Publisher ir_img_pub;

    ros::Publisher rgb_camera_info_pub;
    ros::Publisher rgb_img_raw_pub;
    ros::Publisher mono8_img_pub;
    ros::Publisher mjpeg_img_pub;

    ros::Publisher peak_camera_info_pub;
    ros::Publisher peak_img_pub;

    ros::Publisher depth_odd_raw_pub;
    ros::Publisher depth_points_odd_pub;
    ros::Publisher depth_even_raw_pub;
    ros::Publisher depth_points_even_pub;
    ros::Publisher depth_merge_raw_pub;
    ros::Publisher depth_points_merge_pub;
    ros::Publisher peak_odd_img_pub;
    ros::Publisher peak_even_img_pub;

    AS_CAM_PTR pCamera;
    AS_APP_CAMERA_STATUS_E camStatus;
    int stream_flg;
    AS_CAM_ATTR_S attr_s;
    AS_CONFIG_INFO_S config_info;
    unsigned int index;
} PUBLISHER_INFO_S;

class CameraPublisher : public ICameraStatus
{
public:
    CameraPublisher(ros::NodeHandle nh);
    virtual ~CameraPublisher();

public:
    int start();
    void stop();
    void saveImage();
    void logFps(bool enable);
    bool getLogFps();
    void logCfgParameter();

private:
    virtual int onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param,
                                 const AS_SDK_CAM_MODEL_E &cam_type) override;
    virtual int onCameraDetached(AS_CAM_PTR pCamera) override;
    virtual int onCameraOpen(AS_CAM_PTR pCamera) override;
    virtual int onCameraClose(AS_CAM_PTR pCamera) override;
    virtual int onCameraStart(AS_CAM_PTR pCamera) override;
    virtual int onCameraStop(AS_CAM_PTR pCamera) override;
    virtual void onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData) override;
    virtual void onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData) override;

private:
    int genRosDepthCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::CameraInfo *pstCamInfo,
                           unsigned int seq, AS_CAM_Parameter_s &stParameter, bool registration = false);
    int genRosRgbCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::CameraInfo *pstCamInfo,
                         unsigned int seq, AS_CAM_Parameter_s &stParameter);
    int genRosIrCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::CameraInfo *pstCamInfo,
                        unsigned int seq, AS_CAM_Parameter_s &stParameter, bool registration = false);
    int genRosPeakCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::CameraInfo *pstCamInfo,
                          unsigned int seq, AS_CAM_Parameter_s &stParameter);

    int genRosDepthImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                         unsigned int seq);
    int genRosRgbImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                       unsigned int seq);
    int genRosIrImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                      unsigned int seq);
    int genRosMono8Image(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                         unsigned int seq);
    int genRosPeakImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                        unsigned int seq);
    int genRosMjpegImage(const AS_Frame_s *pstFrame, sensor_msgs::CompressedImage *pstImage,
                         unsigned int seq);
    void depthInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                            AS_CAM_Parameter_s &stParameter, ros::Time time);
    void pointcloudPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pData,
                             AS_CAM_Parameter_s &stParameter);
    void rgbInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                          AS_CAM_Parameter_s &stParameter, ros::Time time);
    void irInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                         AS_CAM_Parameter_s &stParameter, ros::Time time);
    void peakInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                           AS_CAM_Parameter_s &stParameter, ros::Time time);
    void depthMergeInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData,
                                 AS_CAM_Parameter_s &stParameter, ros::Time time);
    void pointcloudMergePublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pData,
                                  AS_CAM_Parameter_s &stParameter);
    void mjpegInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                            AS_CAM_Parameter_s &stParameter, ros::Time time);
    void tfPublisher(AS_CAM_PTR pCamera, AS_CAM_Parameter_s &stParameter);

    int setResolution(AS_CAM_PTR pCamera, LAUNCH_CONFI_PARAM_S resolution);

    void saveImage(const std::string &serialno, const AS_SDK_Data_s *pstData);
    int streamController();
    bool virtualMachine();
    int initLaunchParams();
    void logCameraPathInfo(AS_CAM_ATTR_S &attr_t);
    int printLaunchParams(LAUNCH_CONFI_PARAM_S para);
    int checkStreamContrll(unsigned int allType, unsigned int checkType, AS_CAM_PTR pCamera, int &streamFlag,
                           AS_APP_CAMERA_STATUS_E status);
    void getSubTopicStreamType(const ros::Publisher &publisher, unsigned int &streamType, unsigned int idx);
    void getSubListStreamType(const PUBLISHER_INFO_S &publisherInfo, unsigned int &type);

private:
    std::string m_nodeNameSpace;
    ros::NodeHandle m_nh;
    std::map<AS_CAM_PTR, AS_SDK_CAM_MODEL_E> m_cam_type_map;
    std::list<PUBLISHER_INFO_S> imgPubList;
    CameraSrv *server = nullptr;
    bool m_logfps = false;
    std::unordered_map<AS_CAM_PTR, std::shared_ptr<Camera>> m_camera_map;
    LAUNCH_CONFI_PARAM_S m_launch_param;
    std::thread m_monitor_thd;
    bool m_monitor_flg;
};
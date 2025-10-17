/**
 * @file      CameraPublisher.cpp
 * @brief     angstrong camera publisher.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/02/15
 * @version   1.0

 */

#include "CameraPublisher.h"
#include "ros/ros.h"

#define LOG_PARA(value, str)\
    do { \
        if (value != -1) { \
            ROS_INFO("get %s %d", str, value); \
        } \
    } while(0)


CameraPublisher::CameraPublisher(ros::NodeHandle nh)
{
    m_nh = nh;

    initLaunchParams();
    m_nodeNameSpace = m_nh.getNamespace();
}

CameraPublisher::~CameraPublisher()
{
}

int CameraPublisher::start()
{
    int ret = 0;

    std::string config_path;
    m_nh.param<std::string>("confiPath", config_path, "");
    if (config_path.size() == 0) {
        ROS_ERROR("config file path error, check your file path" );
        return -1;
    } else {
        ROS_INFO("config_path %s", config_path.c_str());
    }

    if (server == nullptr) {
        server = new CameraSrv(this, config_path);
        ret = server->start();
        if (ret != 0) {
            ROS_ERROR("start server failed" );
        }
        m_monitor_flg = true;
        m_monitor_thd = std::thread(&CameraPublisher::streamController, this);
    }

    return 0;
}

void CameraPublisher::stop()
{
    if (server != nullptr) {
        server->stop();
        m_monitor_flg = false;
        if (m_monitor_thd.joinable()) {
            m_monitor_thd.join();
        }
        delete server;
        server = nullptr;
    }

    /* free the map */
    m_camera_map.erase(m_camera_map.begin(), m_camera_map.end());
}

void CameraPublisher::saveImage()
{
    for (auto it = m_camera_map.begin(); it != m_camera_map.end(); it++) {
        it->second->enableSaveImage(true);
    }
}

void CameraPublisher::logFps(bool enable)
{
    m_logfps = enable;
}

bool CameraPublisher::getLogFps()
{
    return m_logfps;
}

int CameraPublisher::onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param,
                                      const AS_SDK_CAM_MODEL_E &cam_type)
{
    int ret = 0;
    AS_CAM_ATTR_S attr_t;
    bool reconnected = false;
    unsigned int dev_idx = imgPubList.size();
    param.open = true;
    memset(&attr_t, 0, sizeof(AS_CAM_ATTR_S));
    ret = AS_SDK_GetCameraAttrs(pCamera, attr_t);
    if (ret != 0) {
        ROS_ERROR("get device path info failed" );
    }

    /* open the specified camera */
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        if ((m_launch_param.usb_bus_num != -1) && (strcmp(m_launch_param.usb_port_nums, "null") != 0)) {
            if ((attr_t.attr.usbAttrs.bnum != m_launch_param.usb_bus_num) ||
                (std::string(attr_t.attr.usbAttrs.port_numbers) != m_launch_param.usb_port_nums)) {
                param.open = false;
                ROS_WARN("found a dev but cannot match the specified camera, launch bus num %d , port num %s , but found bus num %d, port num %s",
                         m_launch_param.usb_bus_num, m_launch_param.usb_port_nums, attr_t.attr.usbAttrs.bnum, attr_t.attr.usbAttrs.port_numbers);
                return -1;
            } else {
                ROS_INFO("open the specified camera : bus num %d, port numbers %s", attr_t.attr.usbAttrs.bnum,
                         attr_t.attr.usbAttrs.port_numbers);
            }
        }
        break;
    case AS_CAMERA_ATTR_NET:
        break;
    default:
        break;
    }
    logCameraPathInfo(attr_t);

    /* create publisher */
    unsigned int imgPubIdx = 0;
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if ((it->attr_s.attr.usbAttrs.bnum == attr_t.attr.usbAttrs.bnum)
                && (strcmp(it->attr_s.attr.usbAttrs.port_numbers, attr_t.attr.usbAttrs.port_numbers) == 0)) {
                ROS_INFO("reconnected, update the camera info" );
                param.start = true;
                it->pCamera = pCamera;
                memcpy(&it->attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
                it->stream_flg = 0;
                reconnected = true;
                dev_idx = imgPubIdx;
                break;
            }
            imgPubIdx++;
        }
        break;
    case AS_CAMERA_ATTR_NET:
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if ((it->attr_s.attr.netAttrs.port == attr_t.attr.netAttrs.port)
                && (strcmp(it->attr_s.attr.netAttrs.ip_addr, attr_t.attr.netAttrs.ip_addr) == 0)) {
                ROS_INFO("reconnected, update the camera info" );
                param.start = true;
                it->pCamera = pCamera;
                memcpy(&it->attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
                it->stream_flg = 0;
                reconnected = true;
                dev_idx = imgPubIdx;
                break;
            }
            imgPubIdx++;
        }
        break;
    default:
        break;
    }

    /* create publisher */
    if (!reconnected) {
        ROS_INFO("create a new publisher info set" );
        PUBLISHER_INFO_S stPublisherInfo;
        stPublisherInfo.index = dev_idx;
        switch (cam_type) {
        case AS_SDK_CAM_MODEL_NUWA_XB40:
        case AS_SDK_CAM_MODEL_NUWA_X100:
        case AS_SDK_CAM_MODEL_NUWA_HP60:
        case AS_SDK_CAM_MODEL_NUWA_HP60V:
            stPublisherInfo.depth_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(dev_idx) + "/image_raw",
                                            100);
            stPublisherInfo.ir_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("ir" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.ir_img_pub = m_nh.advertise<sensor_msgs::Image>("ir" + std::to_string(dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_CHANGJIANG_B:
            stPublisherInfo.depth_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(dev_idx) + "/connectDepth",
                                            100);
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_TANGGULA_A:
            stPublisherInfo.depth_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(dev_idx) + "/image_raw",
                                            100);
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_HP60C:
        case AS_SDK_CAM_MODEL_HP60CN:
        case AS_SDK_CAM_MODEL_TANGGULA:
        case AS_SDK_CAM_MODEL_TAISHAN:
        case AS_SDK_CAM_MODEL_TANGGULA_B:
            stPublisherInfo.depth_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(dev_idx) + "/image_raw",
                                            100);
            stPublisherInfo.rgb_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("rgb" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.rgb_img_raw_pub = m_nh.advertise<sensor_msgs::Image>("rgb" + std::to_string(dev_idx) + "/image", 100);
            if (m_launch_param.pub_mono8) {
                stPublisherInfo.mono8_img_pub = m_nh.advertise<sensor_msgs::Image>("mono8" + std::to_string(dev_idx) + "/image", 100);
            }
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            stPublisherInfo.mjpeg_img_pub = m_nh.advertise<sensor_msgs::CompressedImage>("mjpeg" + std::to_string(
                                                dev_idx) + "/compressed", 100);
            break;
        case AS_SDK_CAM_MODEL_VEGA:
            stPublisherInfo.depth_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(dev_idx) + "/image_raw",
                                            100);
            stPublisherInfo.rgb_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("rgb" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.rgb_img_raw_pub = m_nh.advertise<sensor_msgs::Image>("rgb" + std::to_string(dev_idx) + "/image", 100);
            stPublisherInfo.ir_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("ir" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.ir_img_pub = m_nh.advertise<sensor_msgs::Image>("ir" + std::to_string(dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_KUNLUN_A:
            stPublisherInfo.depth_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_odd_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(
                                                    dev_idx) + "/odd_image_raw", 100);
            stPublisherInfo.depth_points_odd_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                    dev_idx) + "/odd_points", 100);
            stPublisherInfo.depth_even_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(
                    dev_idx) + "/even_image_raw", 100);
            stPublisherInfo.depth_points_even_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                    dev_idx) + "/even_points", 100);
            stPublisherInfo.depth_merge_raw_pub = m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(
                    dev_idx) + "/merge_image_raw", 100);
            stPublisherInfo.depth_points_merge_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                        dev_idx) + "/merge_points", 100);
            stPublisherInfo.peak_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("peak" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.peak_odd_img_pub = m_nh.advertise<sensor_msgs::Image>("peak" + std::to_string(dev_idx) + "/odd_image",
                                               100);
            stPublisherInfo.peak_even_img_pub = m_nh.advertise<sensor_msgs::Image>("peak" + std::to_string(dev_idx) + "/even_image",
                                                100);
            break;
        case AS_SDK_CAM_MODEL_KUNLUN_C:
            stPublisherInfo.depth_camera_info_pub =
                m_nh.advertise<sensor_msgs::CameraInfo>("depth" + std::to_string(dev_idx) + "/camera_info", 100);
            stPublisherInfo.depth_raw_pub =
                m_nh.advertise<sensor_msgs::Image>("depth" + std::to_string(dev_idx) + "/image_raw", 100);
            stPublisherInfo.rgb_camera_info_pub =
                m_nh.advertise<sensor_msgs::CameraInfo>("rgb" + std::to_string(dev_idx) + "/camera_info", 100);
            stPublisherInfo.rgb_img_raw_pub =
                m_nh.advertise<sensor_msgs::Image>("rgb" + std::to_string(dev_idx) + "/image", 100);
            if (m_launch_param.pub_mono8) {
                stPublisherInfo.mono8_img_pub = m_nh.advertise<sensor_msgs::Image>("mono8" + std::to_string(dev_idx) + "/image", 100);
            }
            stPublisherInfo.peak_camera_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("peak" + std::to_string(
                    dev_idx) + "/camera_info", 100);
            stPublisherInfo.peak_img_pub = m_nh.advertise<sensor_msgs::Image>("peak" + std::to_string(dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_KONDYOR:
        case AS_SDK_CAM_MODEL_KONDYOR_NET:
            stPublisherInfo.depth_points_pub = m_nh.advertise<sensor_msgs::PointCloud2>("depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        default:
            break;
        }

        stPublisherInfo.pCamera = pCamera;
        memcpy(&stPublisherInfo.attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
        if (param.image_flag == 0) {
            stPublisherInfo.stream_flg = 0x0fffffff;
        } else {
            stPublisherInfo.stream_flg = param.image_flag;
        }
        stPublisherInfo.camStatus = CAMERA_CLOSED_STATUS;
        imgPubList.push_back(stPublisherInfo);
    }

    m_camera_map.insert(std::make_pair(pCamera, std::make_shared<Camera>(pCamera, cam_type, m_nodeNameSpace, dev_idx)));
    m_cam_type_map.insert(std::make_pair(pCamera, cam_type));

    /* nuwa camera whether match usb device.
       If it is a virtual machine, do not match it. */
    if ((cam_type == AS_SDK_CAM_MODEL_NUWA_XB40) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_X100) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_HP60) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_HP60V)) {
        extern int AS_Nuwa_SetUsbDevMatch(bool is_match);
        AS_Nuwa_SetUsbDevMatch(!virtualMachine());
        // AS_Nuwa_SetUsbDevMatch(false);
    }

    return 0;
}

int CameraPublisher::onCameraDetached(AS_CAM_PTR pCamera)
{
    int ret = 0;

    ROS_INFO("camera detached" );
    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        m_camera_map.erase(pCamera);
    }

    if (m_cam_type_map.find(pCamera) != m_cam_type_map.end()) {
        m_cam_type_map.erase(pCamera);
    }
    return ret;
}

void CameraPublisher::onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData)
{
    int ret = -1;
    std::string serialno = "";

    AS_CAM_Parameter_s stIrRgbParameter;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        if (m_logfps) {
            camIt->second->checkFps();
        }
        camIt->second->getSerialNo(serialno);
        camIt->second->saveImage(pstData);
        ret = camIt->second->getCamParameter(stIrRgbParameter);
    }

    ros::Time time = ros::Time::now();

    if (ret == 0) {
        depthInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        pointcloudPublisher(pCamera, pstData, stIrRgbParameter);
        rgbInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        irInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        peakInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        mjpegInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        tfPublisher(pCamera, stIrRgbParameter);
    }
}

void CameraPublisher::onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData)
{
    int ret = -1;
    std::string serialno = "";

    AS_CAM_Parameter_s stIrRgbParameter;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        if (m_logfps) {
            camIt->second->checkFps();
        }
        camIt->second->getSerialNo(serialno);
        camIt->second->saveMergeImage(pstData);
        ret = camIt->second->getCamParameter(stIrRgbParameter);
    }

    ros::Time time = ros::Time::now();

    if (ret == 0) {
        depthMergeInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        pointcloudMergePublisher(pCamera, pstData, stIrRgbParameter);
    }
}

int CameraPublisher::onCameraOpen(AS_CAM_PTR pCamera)
{
    int ret = 0;
    ROS_INFO("camera opened");

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            memset(&it->config_info, 0, sizeof(AS_CONFIG_INFO_S));
            AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
            ret = AS_SDK_GetCameraModel(pCamera, cam_type);
            if (ret < 0) {
                ROS_ERROR("get camera model fail");
                return ret;
            }
            if ((cam_type == AS_SDK_CAM_MODEL_HP60C) || (cam_type == AS_SDK_CAM_MODEL_HP60CN)
                || (cam_type == AS_SDK_CAM_MODEL_VEGA) || (cam_type == AS_SDK_CAM_MODEL_TAISHAN)
                || (cam_type == AS_SDK_CAM_MODEL_TANGGULA_B)) {
                ret = AS_SDK_GetConfigInfo(pCamera, it->config_info);
                ROS_INFO("get config info, ret %d, is_Registration %d", ret, it->config_info.is_Registration);
            }
            it->camStatus = CAMERA_OPENED_STATUS;
        }
    }

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        camIt->second->init();
    }

    ret = setResolution(pCamera, m_launch_param);
    if (ret < 0) {
        ROS_ERROR("set resolution fail.");
        return ret;
    }

    return ret;
}

int CameraPublisher::onCameraClose(AS_CAM_PTR pCamera)
{
    int ret = 0;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            it->camStatus = CAMERA_CLOSED_STATUS;
            break;
        }
    }

    return ret;
}

int CameraPublisher::onCameraStart(AS_CAM_PTR pCamera)
{
    int ret = 0;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            it->camStatus = CAMERA_STREAM_STATUS;
            break;
        }
    }

    return ret;
}

int CameraPublisher::onCameraStop(AS_CAM_PTR pCamera)
{
    int ret = 0;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            it->camStatus = CAMERA_OPENED_STATUS;
            break;
        }
    }

    return ret;
}

int CameraPublisher::genRosDepthCamInfo(const AS_Frame_s *pstFrame,
                                        sensor_msgs::CameraInfo *pstCamInfo,
                                        unsigned int seq, AS_CAM_Parameter_s &stParameter,
                                        bool registration)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    if (registration) {
        pstCamInfo->K[0] = stParameter.fxrgb;
        pstCamInfo->K[2] = stParameter.cxrgb;
        pstCamInfo->K[4] = stParameter.fyrgb;
        pstCamInfo->K[5] = stParameter.cyrgb;
    } else {
        pstCamInfo->K[0] = stParameter.fxir;
        pstCamInfo->K[2] = stParameter.cxir;
        pstCamInfo->K[4] = stParameter.fyir;
        pstCamInfo->K[5] = stParameter.cyir;
    }
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosRgbCamInfo(const AS_Frame_s *pstFrame,
                                      sensor_msgs::CameraInfo *pstCamInfo,
                                      unsigned int seq, AS_CAM_Parameter_s &stParameter)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    pstCamInfo->K[0] = stParameter.fxrgb;
    pstCamInfo->K[2] = stParameter.cxrgb;
    pstCamInfo->K[4] = stParameter.fyrgb;
    pstCamInfo->K[5] = stParameter.cyrgb;
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosIrCamInfo(const AS_Frame_s *pstFrame,
                                     sensor_msgs::CameraInfo *pstCamInfo,
                                     unsigned int seq, AS_CAM_Parameter_s &stParameter,
                                     bool registration)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    if (registration) {
        pstCamInfo->K[0] = stParameter.fxrgb;
        pstCamInfo->K[2] = stParameter.cxrgb;
        pstCamInfo->K[4] = stParameter.fyrgb;
        pstCamInfo->K[5] = stParameter.cyrgb;
    } else {
        pstCamInfo->K[0] = stParameter.fxir;
        pstCamInfo->K[2] = stParameter.cxir;
        pstCamInfo->K[4] = stParameter.fyir;
        pstCamInfo->K[5] = stParameter.cyir;
    }
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosPeakCamInfo(const AS_Frame_s *pstFrame,
                                       sensor_msgs::CameraInfo *pstCamInfo,
                                       unsigned int seq, AS_CAM_Parameter_s &stParameter)
{
    boost::array<double, 9> R = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1
                                };

    pstCamInfo->header.seq = seq;
    // pstCamInfo->header.stamp = ros::Time::now();

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->D.resize(5);
    pstCamInfo->D[0] = 0;
    pstCamInfo->D[1] = 0;
    pstCamInfo->D[2] = 0;
    pstCamInfo->D[3] = 0;
    pstCamInfo->D[4] = 0;

    pstCamInfo->K.assign(0.0);
    pstCamInfo->K[0] = stParameter.fxir;
    pstCamInfo->K[2] = stParameter.cxir;
    pstCamInfo->K[4] = stParameter.fyir;
    pstCamInfo->K[5] = stParameter.cyir;
    pstCamInfo->K[8] = 1.0;

    pstCamInfo->R = R;

    pstCamInfo->P.assign(0.0);
    pstCamInfo->P[0] = pstCamInfo->K[0];
    pstCamInfo->P[2] = pstCamInfo->K[2];
    pstCamInfo->P[3] = 0.0;
    pstCamInfo->P[5] = pstCamInfo->K[4];
    pstCamInfo->P[6] = pstCamInfo->K[5];
    pstCamInfo->P[7] = 0;
    pstCamInfo->P[10] = 1.0;
    pstCamInfo->P[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosDepthImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                      unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    if (pstFrame->size == pstFrame->width * pstFrame->height * 2) {
        pstImage->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        pstImage->step = pstFrame->width * 2;
    } else {
        pstImage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        pstImage->step = pstFrame->width * 4;
    }
    pstImage->is_bigendian = false;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));

    return 0;
}

int CameraPublisher::genRosRgbImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                    unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::BGR8;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width * 3;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

int CameraPublisher::genRosIrImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                   unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

int CameraPublisher::genRosMono8Image(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                      unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::MONO8;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;
    // std::copy((unsigned char *)pstFrame->data, (unsigned char *)pstFrame->data + pstFrame->size, std::back_inserter(pstImage->data));

    return 0;
}

int CameraPublisher::genRosPeakImage(const AS_Frame_s *pstFrame, sensor_msgs::Image *pstImage,
                                     unsigned int seq)
{
    pstImage->header.seq = seq;
    // pstImage->header.stamp = ros::Time::now();

    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

int CameraPublisher::genRosMjpegImage(const AS_Frame_s *pstFrame, sensor_msgs::CompressedImage *pstImage,
                                      unsigned int seq)
{
    pstImage->format = "jpeg";
    pstImage->header.seq = seq;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

void CameraPublisher::depthInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    static unsigned int seq = 0;
    if (pstData->depthImg.size == 0) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish depth info");
    sensor_msgs::CameraInfo stDepthCamInfo;
    sensor_msgs::Image stDepthImage;

    stDepthCamInfo.header.stamp = time;
    stDepthImage.header.stamp = time;

    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            if (cam_type != AS_SDK_CAM_MODEL_KONDYOR) {
                genRosDepthCamInfo(&pstData->depthImg, &stDepthCamInfo, seq, stParameter, it->config_info.is_Registration);
            }
            genRosDepthImage(&pstData->depthImg, &stDepthImage, seq);
            if (ros::ok()) {
                auto camIt = m_camera_map.find(pCamera);
                if (camIt == m_camera_map.end()) {
                    ROS_ERROR("error camera ptr");
                    return;
                }
                if (m_launch_param.pub_tfTree) {
                    if (it->config_info.is_Registration) {
                        stDepthImage.header.frame_id = camIt->second->getColorFrameId();
                        stDepthCamInfo.header.frame_id = camIt->second->getColorFrameId();
                    } else {
                        stDepthImage.header.frame_id = camIt->second->getDepthFrameId();
                        stDepthCamInfo.header.frame_id = camIt->second->getDepthFrameId();
                    }
                } else {
                    stDepthImage.header.frame_id = camIt->second->getDefaultFrameId();
                    stDepthCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                }

                if (cam_type != AS_SDK_CAM_MODEL_KONDYOR) {
                    it->depth_camera_info_pub.publish(stDepthCamInfo);
                }
                if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                    if (pstData->depthImg.height == 96) {
                        it->depth_odd_raw_pub.publish(stDepthImage);
                    } else if (pstData->depthImg.height == 8) {
                        it->depth_even_raw_pub.publish(stDepthImage);
                    }
                } else {
                    it->depth_raw_pub.publish(stDepthImage);
                }
                ros::spinOnce();
            }
            break;
        }
    }
    seq++;

    return;
}

void CameraPublisher::pointcloudPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pData,
        AS_CAM_Parameter_s &stParameter)
{
    if (pData->pointCloud.size == 0) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish pointcloud");
    sensor_msgs::PointCloud2 pointCloudMsg;
    sensor_msgs::PointCloud2 pointCloudMsgRgb;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    /* publish point cloud */
    if (m_launch_param.color_pcl == false) {
        pcl::PointCloud<pcl::PointXYZ> stPointCloud;
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (it->config_info.is_pclOrganized) {
                    stPointCloud.width = pData->pointCloud.width;
                    stPointCloud.height = pData->pointCloud.height;
                } else {
                    stPointCloud.width = pData->pointCloud.size / sizeof(float) / 3;
                    stPointCloud.height = 1;
                }
            }
        }
        stPointCloud.points.resize(stPointCloud.width * stPointCloud.height);

        for (unsigned int i = 0 ; i < stPointCloud.points.size(); i++) {
            int index = i * 3;
            stPointCloud.points[i].x = *((float *)pData->pointCloud.data + index) / 1000;
            stPointCloud.points[i].y = *((float *)pData->pointCloud.data + index + 1) / 1000;
            stPointCloud.points[i].z = *((float *)pData->pointCloud.data + index + 2) / 1000;
        }

        static unsigned int seq = 0;
        pcl::toROSMsg(stPointCloud, pointCloudMsg);
        pointCloudMsg.header.stamp = ros::Time::now();
        pointCloudMsg.header.seq = seq++;

        AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
        auto it_par = m_cam_type_map.find(pCamera);
        if ( it_par != m_cam_type_map.end()) {
            cam_type = it_par->second;
        }
        if (ros::ok()) {
            for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
                if (it->pCamera == pCamera) {
                    if (m_launch_param.pub_tfTree) {
                        if (it->config_info.is_Registration) {
                            pointCloudMsg.header.frame_id = camIt->second->getColorFrameId();
                        } else {
                            pointCloudMsg.header.frame_id = camIt->second->getDepthFrameId();
                        }
                    } else {
                        pointCloudMsg.header.frame_id =  camIt->second->getDefaultFrameId();
                    }

                    if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                        if (pData->pointCloud.height == 96) {
                            it->depth_points_odd_pub.publish(pointCloudMsg);
                        } else if (pData->pointCloud.height == 8) {
                            it->depth_points_even_pub.publish(pointCloudMsg);
                        }
                    } else {
                        it->depth_points_pub.publish(pointCloudMsg);
                    }
                }
            }
            ros::spinOnce();
        }
    } else { /* publish point cloud with rgb */
        static unsigned int color_pcl_seq = 0;
        pcl::PointCloud<pcl::PointXYZRGB> stPointCloudRgb;
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (it->config_info.is_pclOrganized) {
                    stPointCloudRgb.width = pData->pointCloud.width;
                    stPointCloudRgb.height = pData->pointCloud.height;
                } else {
                    stPointCloudRgb.width = pData->pointCloud.size / sizeof(float) / 3;
                    stPointCloudRgb.height = 1;
                }
            }
        }
        bool decimal = (pData->depthImg.size == pData->depthImg.width * pData->depthImg.height * 2) ? false : true;
        for (unsigned int y = 0; y < pData->depthImg.height; y++) {
            for (unsigned int x = 0; x < pData->depthImg.width; x++) {
                double depth = 0.0;
                if (decimal) {
                    depth = ((float *)pData->depthImg.data)[y * pData->depthImg.width + x];
                } else {
                    depth = ((uint16_t *)pData->depthImg.data)[y * pData->depthImg.width + x];
                }
                if (depth < 0.0001)
                    continue;
                pcl::PointXYZRGB p;

                p.z = depth / 1000;
                p.x = (x - stParameter.cxrgb) * p.z / stParameter.fxrgb;
                p.y = (y - stParameter.cyrgb) * p.z / stParameter.fyrgb;

                p.b = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x) * 3 ];
                p.g = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x) * 3 + 1];
                p.r = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x ) * 3 + 2];

                stPointCloudRgb.points.push_back(p);
            }
        }
        pcl::toROSMsg(stPointCloudRgb, pointCloudMsgRgb);
        pointCloudMsgRgb.header.stamp = ros::Time::now();
        pointCloudMsgRgb.header.seq = color_pcl_seq++;
        if (ros::ok()) {
            for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
                if (it->pCamera == pCamera) {
                    if (m_launch_param.pub_tfTree) {
                        if (it->config_info.is_Registration) {
                            pointCloudMsgRgb.header.frame_id = camIt->second->getColorFrameId();
                        } else {
                            pointCloudMsgRgb.header.frame_id = camIt->second->getDepthFrameId();
                        }
                    } else {
                        pointCloudMsgRgb.header.frame_id = camIt->second->getDefaultFrameId();
                    }
                    it->depth_points_pub.publish(pointCloudMsgRgb);
                }
            }
            ros::spinOnce();
        }
    }
}

void CameraPublisher::rgbInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                       AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    if (pstData->rgbImg.size == 0) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish color(rgb)");
    static unsigned int seq = 0;
    sensor_msgs::CameraInfo stRgbCamInfo;
    sensor_msgs::Image rgbImage;
    sensor_msgs::Image mono8Image;

    stRgbCamInfo.header.stamp = time;
    rgbImage.header.stamp = time;

    genRosRgbCamInfo(&pstData->rgbImg, &stRgbCamInfo, seq, stParameter);
    genRosRgbImage(&pstData->rgbImg, &rgbImage, seq);
    if (m_launch_param.pub_mono8) {
        unsigned int img_width = pstData->rgbImg.width;
        unsigned int img_height = pstData->rgbImg.height;
        std::shared_ptr<unsigned char> ptr(new (std::nothrow) unsigned char[img_width * img_height],
                                           std::default_delete<unsigned char[]>());
        unsigned char *mono8_data = ptr.get();;
        int ret = bgr2mono8(mono8_data, static_cast<unsigned char *>(pstData->rgbImg.data), img_width, img_height);
        if (ret == 0) {
            AS_Frame_s mono8_frame;
            mono8_frame.data = static_cast<void *>(mono8_data);
            mono8_frame.width = pstData->rgbImg.width;
            mono8_frame.height = pstData->rgbImg.height;
            mono8_frame.size = pstData->rgbImg.width * pstData->rgbImg.height;
            genRosMono8Image(&mono8_frame, &mono8Image, seq);
        }
    }
    seq++;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stRgbCamInfo.header.frame_id = camIt->second->getColorFrameId();
                    rgbImage.header.frame_id = camIt->second->getColorFrameId();
                } else {
                    stRgbCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                    rgbImage.header.frame_id = camIt->second->getDefaultFrameId();
                }

                it->rgb_camera_info_pub.publish(stRgbCamInfo);
                it->rgb_img_raw_pub.publish(rgbImage);

                if (m_launch_param.pub_mono8) {
                    if (m_launch_param.pub_tfTree) {
                        mono8Image.header.frame_id = camIt->second->getColorFrameId();
                    } else {
                        mono8Image.header.frame_id = camIt->second->getDefaultFrameId();
                    }
                    it->mono8_img_pub.publish(mono8Image);
                }
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::irInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                      AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    static unsigned int seq = 0;
    if (pstData->irImg.size == 0) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish infrared");
    sensor_msgs::CameraInfo stIrCamInfo;
    sensor_msgs::Image irImage;
    genRosIrCamInfo(&pstData->irImg, &stIrCamInfo, seq, stParameter);
    genRosIrImage(&pstData->irImg, &irImage, seq);
    seq++;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }
    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stIrCamInfo.header.frame_id = camIt->second->getDepthFrameId();
                    irImage.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    stIrCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                    irImage.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->ir_camera_info_pub.publish(stIrCamInfo);
                it->ir_img_pub.publish(irImage);
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::peakInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    static unsigned int seq = 0;
    if (pstData->peakImg.size == 0) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish peak");
    sensor_msgs::CameraInfo stPeakCamInfo;
    sensor_msgs::Image peakImage;
    genRosPeakCamInfo(&pstData->peakImg, &stPeakCamInfo, seq, stParameter);
    genRosPeakImage(&pstData->peakImg, &peakImage, seq);
    seq++;

    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if (it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stPeakCamInfo.header.frame_id = camIt->second->getDepthFrameId();
                    peakImage.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    stPeakCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                    peakImage.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->peak_camera_info_pub.publish(stPeakCamInfo);
                if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                    if (pstData->peakImg.height == 96) {
                        it->peak_odd_img_pub.publish(peakImage);
                    } else if (pstData->peakImg.height == 8) {
                        it->peak_even_img_pub.publish(peakImage);
                    }
                } else {
                    it->peak_img_pub.publish(peakImage);
                }
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::depthMergeInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData,
        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }
    if (cam_type != AS_SDK_CAM_MODEL_KUNLUN_A) {
        return;
    }

    static unsigned int seq = 0;
    if (pstData->depthImg.size == 0) {
        return;
    }

    sensor_msgs::Image stDepthImage;
    stDepthImage.header.stamp = time;
    genRosDepthImage(&pstData->depthImg, &stDepthImage, seq);
    seq++;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stDepthImage.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    stDepthImage.header.frame_id = camIt->second->getDefaultFrameId();
                }

                it->depth_merge_raw_pub.publish(stDepthImage);
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::pointcloudMergePublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pData,
        AS_CAM_Parameter_s &stParameter)
{
    if (pData->pointCloud.size == 0) {
        return;
    }
    sensor_msgs::PointCloud2 pointCloudMsg;
    sensor_msgs::PointCloud2 pointCloudMsgRgb;

    /* publish point cloud */
    pcl::PointCloud<pcl::PointXYZ> stPointCloud;
    stPointCloud.width = pData->pointCloud.size / sizeof(float) / 3;
    stPointCloud.height = 1;
    stPointCloud.points.resize(stPointCloud.width * stPointCloud.height);

    for (unsigned int i = 0 ; i < stPointCloud.points.size(); i++) {
        int index = i * 3;
        stPointCloud.points[i].x = *((float *)pData->pointCloud.data + index) / 1000;
        stPointCloud.points[i].y = *((float *)pData->pointCloud.data + index + 1) / 1000;
        stPointCloud.points[i].z = *((float *)pData->pointCloud.data + index + 2) / 1000;
    }

    static unsigned int merge_seq = 0;
    pcl::toROSMsg(stPointCloud, pointCloudMsg);
    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudMsg.header.seq = merge_seq++;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    pointCloudMsg.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    pointCloudMsg.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->depth_points_merge_pub.publish(pointCloudMsg);
            }
        }
        ros::spinOnce();
    }
}

void CameraPublisher::tfPublisher(AS_CAM_PTR pCamera, AS_CAM_Parameter_s &stParameter)
{
    if (!m_launch_param.pub_tfTree) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish tf");
    static unsigned int seq = 0;
    static tf2_ros::StaticTransformBroadcaster br;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            auto it_par = m_cam_type_map.find(pCamera);
            if (it_par != m_cam_type_map.end()) {
                if ((it_par->second == AS_SDK_CAM_MODEL_HP60C) || (it_par->second == AS_SDK_CAM_MODEL_HP60CN) ||
                    (it_par->second == AS_SDK_CAM_MODEL_VEGA) || (it_par->second == AS_SDK_CAM_MODEL_KUNLUN_C) ||
                    (it_par->second == AS_SDK_CAM_MODEL_KONDYOR) || (it_par->second == AS_SDK_CAM_MODEL_KONDYOR_NET) ||
                    (it_par->second == AS_SDK_CAM_MODEL_CHANGA) || (it_par->second == AS_SDK_CAM_MODEL_CHANGJIANG_B) ||
                    (it_par->second == AS_SDK_CAM_MODEL_TANGGULA) || (it_par->second == AS_SDK_CAM_MODEL_TAISHAN) ||
                    (it_par->second == AS_SDK_CAM_MODEL_TANGGULA_B)) {
                    geometry_msgs::TransformStamped rgbTransform;
                    rgbTransform.header.seq = seq++;
                    rgbTransform.header.stamp = ros::Time::now();
                    rgbTransform.header.frame_id = camIt->second->getCamLinkFrameId();
                    rgbTransform.child_frame_id = camIt->second->getColorFrameId();
                    rgbTransform.transform.translation.x = stParameter.T1 / 1000;
                    rgbTransform.transform.translation.y = stParameter.T2 / 1000;
                    rgbTransform.transform.translation.z = stParameter.T3 / 1000;
                    rgbTransform.transform.rotation.x = 0;
                    rgbTransform.transform.rotation.y = 0;
                    rgbTransform.transform.rotation.z = 0;

                    tf2::Quaternion qtn;

                    double sy = std::sqrt(stParameter.R00 * stParameter.R00 + stParameter.R10 * stParameter.R10);
                    double angleX;
                    double angleY;
                    double angleZ;
                    bool singular = sy < 1e-6;
                    if (!singular) {
                        angleX = std::atan2(stParameter.R21, stParameter.R22);
                        angleY = std::atan2(-stParameter.R20, sy);
                        angleZ = std::atan2(stParameter.R10, stParameter.R00);
                    } else {
                        angleX = std::atan2(-stParameter.R12, stParameter.R11);
                        angleY = std::atan2(-stParameter.R20, sy);
                        angleZ = 0;
                    }
                    // ROS_INFO("angleX: %lf", angleX);
                    // ROS_INFO("angleY: %lf", angleY);
                    // ROS_INFO("angleZ: %lf", angleZ);

                    qtn.setRPY(angleX, angleY, angleZ);
                    rgbTransform.transform.rotation.x = qtn.getX();
                    rgbTransform.transform.rotation.y = qtn.getY();
                    rgbTransform.transform.rotation.z = qtn.getZ();
                    rgbTransform.transform.rotation.w = qtn.getW();

                    br.sendTransform(rgbTransform);
                }
            }

            geometry_msgs::TransformStamped depthTransform;
            depthTransform.header.stamp = ros::Time::now();
            depthTransform.header.frame_id = camIt->second->getCamLinkFrameId();
            if (it->config_info.is_Registration) {
                depthTransform.child_frame_id = camIt->second->getColorFrameId();
            } else {
                depthTransform.child_frame_id = camIt->second->getDepthFrameId();
            }
            depthTransform.transform.translation.x = 0.0;
            depthTransform.transform.translation.y = 0.0;
            depthTransform.transform.translation.z = 0.0;
            depthTransform.transform.rotation.x = 0.0;
            depthTransform.transform.rotation.y = 0.0;
            depthTransform.transform.rotation.z = 0.0;
            depthTransform.transform.rotation.w = 1.0;

            br.sendTransform(depthTransform);
        }
    }

    ros::spinOnce();
}

void CameraPublisher::mjpegInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
        AS_CAM_Parameter_s &stParameter, ros::Time time)
{
    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        ROS_ERROR("error camera ptr");
        return;
    }

    if (pstData->mjpegImg.size == 0) {
        return;
    }
    ROS_INFO_STREAM_ONCE("publish mjpeg");
    static unsigned int seq = 0;
    sensor_msgs::CompressedImage mjpegImage;

    mjpegImage.header.stamp = time;

    genRosMjpegImage(&pstData->mjpegImg, &mjpegImage, seq);
    seq++;

    if (ros::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    mjpegImage.header.frame_id = camIt->second->getColorFrameId();
                } else {
                    mjpegImage.header.frame_id = camIt->second->getDefaultFrameId();
                }

                it->mjpeg_img_pub.publish(mjpegImage);
            }
        }
        ros::spinOnce();
    }
}


int CameraPublisher::setResolution(AS_CAM_PTR pCamera, LAUNCH_CONFI_PARAM_S resolution)
{
    int ret = 0;
    if ((resolution.set_depth_width != -1) && (resolution.set_depth_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s depthInfo;
        depthInfo.width = resolution.set_depth_width;
        depthInfo.height = resolution.set_depth_height;
        depthInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_DEPTH, &depthInfo);
        if (ret < 0) {
            ROS_ERROR("set depth param fail");
            return ret;
        }
        ROS_INFO("set depth resolution: %dx%d@%dfps", depthInfo.width, depthInfo.height, depthInfo.fps);
    }
    if ((resolution.set_rgb_width != -1) && (resolution.set_rgb_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s rgbInfo;
        rgbInfo.width = resolution.set_rgb_width;
        rgbInfo.height = resolution.set_rgb_height;
        rgbInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_RGB, &rgbInfo);
        if (ret < 0) {
            ROS_ERROR("set rgb param fail");
            return ret;
        }
        ROS_INFO("set rgb resolution: %dx%d@%dfps", rgbInfo.width, rgbInfo.height, rgbInfo.fps);
    }
    if ((resolution.set_ir_width != -1) && (resolution.set_ir_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s irInfo;
        irInfo.width = resolution.set_ir_width;
        irInfo.height = resolution.set_ir_height;
        irInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_IR, &irInfo);
        if (ret < 0) {
            ROS_ERROR("set ir param fail");
            return ret;
        }
        ROS_INFO("set ir resolution: %dx%d@%dfps", irInfo.width, irInfo.height, irInfo.fps);
    }
    if ((resolution.set_peak_width != -1) && (resolution.set_peak_height != -1) && (resolution.set_fps != -1)) {
        AS_STREAM_Param_s peakInfo;
        peakInfo.width = resolution.set_peak_width;
        peakInfo.height = resolution.set_peak_height;
        peakInfo.fps = resolution.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_PEAK, &peakInfo);
        if (ret < 0) {
            ROS_ERROR("set peak param fail");
            return ret;
        }
        ROS_INFO("set peak resolution: %dx%d@%dfps", peakInfo.width, peakInfo.height, peakInfo.fps);
    }
    return ret;
}

void CameraPublisher::saveImage(const std::string &serialno, const AS_SDK_Data_s *pstData)
{

    static int depthindex = 0;
    static int pointCloudIndex = 0;
    static int rgbindex = 0;
    static int irindex = 0;
    static int peakindex = 0;

    if (pstData->depthImg.size > 0) {
        std::string depthImgName(std::string("depth_" + std::to_string(pstData->depthImg.width) + "x" +
                                             std::to_string(pstData->depthImg.height)
                                             + "_" + std::to_string(depthindex++) + ".yuv"));
        if (saveYUVImg(depthImgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            ROS_ERROR("save img failed!");
        } else {
            ROS_INFO("save depth image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), depthImgName.c_str());
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string("PointCloud_" + std::to_string(pointCloudIndex++) + ".txt"));
        if (savePointCloud(pointCloudName.c_str(), (float *)pstData->pointCloud.data,
                           pstData->pointCloud.size / sizeof(float)) != 0) {
            ROS_ERROR("save point cloud failed!");
        } else {
            ROS_INFO("save point cloud success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), pointCloudName.c_str());
        }
    }

    if (pstData->rgbImg.size > 0) {
        std::string rgbName(std::string("rgb_" + std::to_string(pstData->rgbImg.width) + "x" +
                                        std::to_string(pstData->rgbImg.height)
                                        + "_" + std::to_string(rgbindex++) + ".yuv"));
        if (saveYUVImg(rgbName.c_str(), pstData->rgbImg.data, pstData->rgbImg.size) != 0) {
            ROS_ERROR("save rgb image failed!");
        } else {
            ROS_INFO("save rgb image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), rgbName.c_str());
        }
    }

    if (pstData->irImg.size > 0) {
        std::string irName(std::string("ir_" + std::to_string(pstData->irImg.width) + "x" +
                                       std::to_string(pstData->irImg.height)
                                       + "_" + std::to_string(irindex++) + ".yuv"));
        if (saveYUVImg(irName.c_str(), pstData->irImg.data, pstData->irImg.size) != 0) {
            ROS_ERROR("save ir image failed!");
        } else {
            ROS_INFO("save ir image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), irName.c_str());
        }
    }

    if (pstData->peakImg.size > 0) {
        std::string peakName(std::string("peak_" + std::to_string(pstData->peakImg.width) + "x" +
                                         std::to_string(pstData->peakImg.height)
                                         + "_" + std::to_string(peakindex++) + ".yuv"));
        if (saveYUVImg(peakName.c_str(), pstData->peakImg.data, pstData->peakImg.size) != 0) {
            ROS_ERROR("save peak image failed!");
        } else {
            ROS_INFO("save peak image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), peakName.c_str());
        }
    }

}

void CameraPublisher::logCfgParameter()
{
    for (auto it = m_camera_map.begin(); it != m_camera_map.end(); it++) {
        AS_SDK_LogCameraCfg(it->first);
    }
}

void CameraPublisher::getSubTopicStreamType(const ros::Publisher &publisher, unsigned int &streamType, unsigned int idx)
{
    if (publisher) {
        if (publisher.getNumSubscribers()) {
            std::string topic_name = publisher.getTopic();
            if ((topic_name.find("depth" + std::to_string(idx)) != std::string::npos) &&
                topic_name.find("points") != std::string::npos) {
                streamType = POINTCLOUD_IMG_FLG | DEPTH_IMG_FLG;
            } else if ((topic_name.find("connectDepth" + std::to_string(idx)) != std::string::npos)) {
                streamType = DEPTH_IMG_FLG | SUB_DEPTH_IMG_FLG;
            } else if ((topic_name.find("connectDepth" + std::to_string(idx)) != std::string::npos) &&
                       topic_name.find("points") != std::string::npos) {
                streamType = POINTCLOUD_IMG_FLG | DEPTH_IMG_FLG | SUB_DEPTH_IMG_FLG;
            } else if (topic_name.find("depth" + std::to_string(idx)) != std::string::npos) {
                streamType = DEPTH_IMG_FLG;
            } else if (topic_name.find("ir" + std::to_string(idx)) != std::string::npos) {
                streamType = IR_IMG_FLG;
            } else if ((topic_name.find("rgb" + std::to_string(idx)) != std::string::npos)
                       || (topic_name.find("mono8" + std::to_string(idx)) != std::string::npos)) {
                streamType = RGB_IMG_FLG;
            } else if (topic_name.find("peak" + std::to_string(idx)) != std::string::npos) {
                streamType = PEAK_IMG_FLG;
            } else if (topic_name.find("yuyv" + std::to_string(idx)) != std::string::npos) {
                streamType = YUYV_IMG_FLG;
            } else if (topic_name.find("mjpeg" + std::to_string(idx)) != std::string::npos) {
                streamType = MJPEG_IMG_FLG;
            } else {
                streamType = 0;
            }
        } else {
            streamType = 0;
        }
    } else {
        streamType = 0;
    }

    return;
}

void CameraPublisher::getSubListStreamType(const PUBLISHER_INFO_S &publisherInfo, unsigned int &type)
{
    unsigned int streamType = 0;
    type = 0;

    // depth
    getSubTopicStreamType(publisherInfo.depth_camera_info_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_odd_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_even_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_merge_raw_pub, streamType, publisherInfo.index);
    type |= streamType;

    // ir
    getSubTopicStreamType(publisherInfo.ir_camera_info_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.ir_img_pub, streamType, publisherInfo.index);
    type |= streamType;

    // rgb
    getSubTopicStreamType(publisherInfo.rgb_camera_info_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.rgb_img_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.mono8_img_pub, streamType, publisherInfo.index);
    type |= streamType;

    // mjpeg
    getSubTopicStreamType(publisherInfo.mjpeg_img_pub, streamType, publisherInfo.index);
    type |= streamType;

    // peak
    getSubTopicStreamType(publisherInfo.peak_camera_info_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.peak_img_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.peak_odd_img_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.peak_even_img_pub, streamType, publisherInfo.index);
    type |= streamType;

    // point
    getSubTopicStreamType(publisherInfo.depth_points_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_points_odd_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_points_even_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType(publisherInfo.depth_points_merge_pub, streamType, publisherInfo.index);
    type |= streamType;

    return;
}

int CameraPublisher::checkStreamContrll(unsigned int allType, unsigned int checkType, AS_CAM_PTR pCamera,
                                        int &streamFlag, AS_APP_CAMERA_STATUS_E status)
{
    int ret = 0;

    if ((allType & checkType) == checkType) {
        if (!((streamFlag & checkType) == checkType)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (status != CAMERA_CLOSED_STATUS) {
                if ((m_cam_type_map[pCamera] == AS_SDK_CAM_MODEL_E::AS_SDK_CAM_MODEL_CHANGJIANG_B) ||
                    (m_cam_type_map[pCamera] == AS_SDK_CAM_MODEL_E::AS_SDK_CAM_MODEL_TAISHAN)) {
                    AS_SDK_StopStream(pCamera);
                }
                ret = AS_SDK_StartStream(pCamera, streamFlag | checkType);
                // ROS_INFO("AS_SDK_StartStream ret %d, streamType 0x%x", ret, checkType);
                if (ret == 0) {
                    onCameraStart(pCamera);
                    streamFlag |= checkType;
                }
            } else {
                streamFlag |= checkType;
            }
        }
    } else {
        if (allType == 0) {
            if (status == CAMERA_STREAM_STATUS) {
                ret = AS_SDK_StopStream(pCamera);
                if (ret == 0) {
                    streamFlag = 0;
                    onCameraStop(pCamera);
                }
            } else {
                streamFlag = 0;
            }
        } else if ((streamFlag & checkType) == checkType) {
            if (status == CAMERA_STREAM_STATUS) {
                ret = AS_SDK_StopStream(pCamera, checkType);
                // ROS_INFO("AS_SDK_StopStream ret %d, streamType 0x%x", ret, checkType);
                if (ret == 0) {
                    streamFlag ^= checkType;
                    if (streamFlag == 0) {
                        onCameraStop(pCamera);
                    }
                }
            } else {
                streamFlag ^= checkType;
            }
        }
    }

    return ret;
}

int CameraPublisher::streamController()
{
    int ret = 0;
    while (ros::ok()) {
        server->getLock().lock();
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            unsigned int type = 0;
            getSubListStreamType(*it, type);

            // subscrib depth stream
            ret = checkStreamContrll(type, DEPTH_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, RGB_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, MJPEG_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, IR_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, POINTCLOUD_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, YUYV_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, PEAK_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, SUB_DEPTH_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
        }

        server->getLock().unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return ret;
}

bool CameraPublisher::virtualMachine()
{
    int cnt = 0;
    char szCnt[8];
    FILE *fp = nullptr;

    char cmd[128];
    snprintf(cmd, sizeof(cmd) - 1, R"(lscpu | grep "Hypervisor vendor" | wc -l)");
    fp = popen(cmd, "r");
    if (fgets(szCnt, sizeof(szCnt), fp) != nullptr) {
        if (strlen(szCnt) != 0) {
            cnt = std::stoi(szCnt);
        }
    }
    pclose(fp);
    fp = nullptr;
    if (cnt == 0) {
        return false;
    } else {
        return true;
    }
}

void CameraPublisher::logCameraPathInfo(AS_CAM_ATTR_S &attr_t)
{
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        ROS_INFO("usb camera");
        ROS_INFO("bnum: %d", attr_t.attr.usbAttrs.bnum);
        ROS_INFO("dnum: %d", attr_t.attr.usbAttrs.dnum);
        ROS_INFO("port_numbers: %s", attr_t.attr.usbAttrs.port_numbers);
        break;
    case AS_CAMERA_ATTR_NET:
        ROS_INFO("net camera");
        ROS_INFO("ip: %s", attr_t.attr.netAttrs.ip_addr);
        ROS_INFO("port: %d", attr_t.attr.netAttrs.port);
        break;
    default:
        break;
    }
}

int CameraPublisher::initLaunchParams()
{
    memset(&m_launch_param, 0, sizeof(LAUNCH_CONFI_PARAM_S));
    m_nh.param("depth_width", m_launch_param.set_depth_width, -1);
    m_nh.param("depth_height", m_launch_param.set_depth_height, -1);
    m_nh.param("rgb_width", m_launch_param.set_rgb_width, -1);
    m_nh.param("rgb_height", m_launch_param.set_rgb_height, -1);
    m_nh.param("ir_width", m_launch_param.set_ir_width, -1);
    m_nh.param("ir_height", m_launch_param.set_ir_height, -1);
    m_nh.param("peak_width", m_launch_param.set_peak_width, -1);
    m_nh.param("peak_height", m_launch_param.set_peak_height, -1);
    m_nh.param("fps", m_launch_param.set_fps, -1);
    m_nh.param("usb_bus_no", m_launch_param.usb_bus_num, -1);
    std::string usb_port_nums;
    m_nh.param<std::string>("usb_path", usb_port_nums, "null");
    strncpy(m_launch_param.usb_port_nums, usb_port_nums.c_str(),
            std::min(sizeof(m_launch_param.usb_port_nums), usb_port_nums.length()));
    m_nh.param("color_pcl", m_launch_param.color_pcl, false);
    m_nh.param("pub_mono8", m_launch_param.pub_mono8, false);
    m_nh.param("pub_tfTree", m_launch_param.pub_tfTree, true);
    printLaunchParams(m_launch_param);

    return 0;
}

int CameraPublisher::printLaunchParams(LAUNCH_CONFI_PARAM_S para)
{
    LOG_PARA(para.set_depth_width, "depth_width");
    LOG_PARA(para.set_depth_height, "depth_height");

    LOG_PARA(para.set_rgb_width, "rgb_width");
    LOG_PARA(para.set_rgb_height, "rgb_height");

    LOG_PARA(para.set_ir_width, "ir_width");
    LOG_PARA(para.set_ir_height, "ir_height");

    LOG_PARA(para.set_peak_width, "peak_width");
    LOG_PARA(para.set_peak_height, "peak_height");

    LOG_PARA(para.set_fps, "set_fps");
    LOG_PARA(para.usb_bus_num, "usb_bus_num");

    if (strncmp(para.usb_port_nums, "null", strlen("null")) != 0) {
        ROS_INFO("get usb_port_nums %s", para.usb_port_nums);
    }

    if (para.color_pcl != false) {
        ROS_INFO("get color_pcl %d", para.color_pcl);
    }

    if (para.pub_mono8 != false) {
        ROS_INFO("get pub_mono8 %d", para.pub_mono8);
    }

    if (para.pub_tfTree != false) {
        ROS_INFO("get pub_tfTree %d", para.pub_tfTree);
    }

    return 0;
}



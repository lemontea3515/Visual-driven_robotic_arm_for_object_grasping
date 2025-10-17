/**
 * @file      Camera.cpp
 * @brief     angstrong Camera.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/05/29
 * @version   1.0

 */
#include <algorithm>
#include <unistd.h>
#include "Camera.h"
#include "ros/ros.h"

Camera::Camera(AS_CAM_PTR pCamera, const AS_SDK_CAM_MODEL_E &cam_type, const std::string &nodeNameSpace,
               unsigned int devId)
{
    int ret = 0;
    m_handle = pCamera;
    m_cam_type = cam_type;
    m_check_fps = new CheckFps(pCamera);
    ret = AS_SDK_GetCameraAttrs(m_handle,  m_attr);
    if (ret != 0) {
        ROS_WARN("get camera attrs failed");
    }
    memset(&m_cam_parameter, 0, sizeof(AS_CAM_Parameter_s));

    m_frameIdInfo = new TfTreeFrameIdInfo(nodeNameSpace, devId);
}

Camera::~Camera()
{
    if (m_check_fps != nullptr) {
        delete m_check_fps;
        m_check_fps = nullptr;
    }
    if (m_is_thread) {
        m_is_thread = false;
    }
    if (m_backgroundThread.joinable()) {
        m_backgroundThread.join();
    }

    delete m_frameIdInfo;
}

int Camera::init()
{
    int ret = 0;
    char sn_buff[64] = {0};
    ret = AS_SDK_GetSerialNumber(m_handle, sn_buff, sizeof(sn_buff));
    if (ret != 0) {
        ROS_ERROR("get camera serial number failed");
        return -1;
    }
    m_serialno = std::string(sn_buff);

    char fwVersion[100] = {0};
    ret = AS_SDK_GetFwVersion(m_handle, fwVersion, sizeof(fwVersion));
    if (ret == 0) {
        ROS_INFO("#camera[%p] SN[%s]'s firmware version: %s", m_handle, m_serialno.c_str(), fwVersion);
    }
    m_is_thread = true;
    m_backgroundThread = std::thread(&Camera::backgroundThread, this);
    return ret;
}

double Camera::checkFps()
{
    std::string Info = "";
    switch (m_attr.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        Info = (std::to_string(m_attr.attr.usbAttrs.bnum) + ":" + m_attr.attr.usbAttrs.port_numbers);
        break;
    case AS_CAMERA_ATTR_NET:
        Info = (std::to_string(m_attr.attr.netAttrs.port) + ":" + m_attr.attr.netAttrs.ip_addr);
        break;
    case AS_CAMERA_ATTR_WIN_USB:
        Info = std::string(m_attr.attr.winAttrs.symbol_link) + ":" + std::string(m_attr.attr.winAttrs.location_path);
        break;
    default:
        ROS_ERROR("attr type error");
        break;
    }
    return m_check_fps->checkFps(m_serialno, Info);
}

int Camera::enableSaveImage(bool enable)
{
    m_save_img = enable;
    if (m_cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
        m_save_merge_img = enable;
    }
    return 0;
}

int Camera::getSerialNo(std::string &sn)
{
    sn = m_serialno;
    return 0;
}

int Camera::getCameraAttrs(AS_CAM_ATTR_S &info)
{
    info = m_attr;
    return 0;
}

int Camera::getCamParameter(AS_CAM_Parameter_s &cam_parameter)
{
    if (m_isGetParameter) {
        memcpy(&cam_parameter, &m_cam_parameter, sizeof(AS_CAM_Parameter_s));
        return 0;
    } else {
        return -1;
    }
}

int Camera::backgroundThread()
{
    int ret = 0;
    while (m_is_thread) {
        // KONDYOR not support to get CamParameter, KUNLUN A don't need to get CamParameter
        if ((m_cam_type != AS_SDK_CAM_MODEL_KONDYOR_NET) && (m_cam_type != AS_SDK_CAM_MODEL_KONDYOR)) {
            ret = AS_SDK_GetCamParameter(m_handle, &m_cam_parameter);
            if (ret == 0) {
                ROS_INFO("SN [%s]'s parameter:", m_serialno.c_str());
                ROS_INFO("irfx: %lf", m_cam_parameter.fxir);
                ROS_INFO("irfy: %lf", m_cam_parameter.fyir);
                ROS_INFO("ircx: %lf", m_cam_parameter.cxir);
                ROS_INFO("ircy: %lf", m_cam_parameter.cyir);
                ROS_INFO("rgbfx: %lf", m_cam_parameter.fxrgb);
                ROS_INFO("rgbfy: %lf", m_cam_parameter.fyrgb);
                ROS_INFO("rgbcx: %lf", m_cam_parameter.cxrgb);
                ROS_INFO("rgbcy: %lf", m_cam_parameter.cyrgb);
                m_is_thread = false;
                m_isGetParameter = true;
                break;
            }
        } else {
            m_isGetParameter = true;
            m_is_thread = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}

void Camera::saveImage(const AS_SDK_Data_s *pstData)
{
    if (!m_save_img) {
        m_cnt = 0;
        return;
    }

    if (m_cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
        if (m_cnt >= 1) {
            m_save_img = false;
        }
        m_cnt++;
    } else {
        m_save_img = false;
    }

    if (pstData->depthImg.size > 0) {
        std::string depthimgName(std::string(m_serialno + "_depth_") + std::to_string(
                                     pstData->depthImg.width) + "x" + std::to_string(pstData->depthImg.height)
                                 + "_" + std::to_string(m_depthindex++) + ".yuv");
        if (saveYUVImg(depthimgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            ROS_ERROR("save depth image failed!");
        } else {
            ROS_INFO("save depth image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), depthimgName.c_str());
        }
    }

    if (pstData->rgbImg.size > 0) {
        std::string rgbName(std::string(m_serialno + "_rgb_") + std::to_string(pstData->rgbImg.width) + "x" +
                            std::to_string(pstData->rgbImg.height) + "_" + std::to_string(m_rgbindex++) + ".yuv");
        if (saveYUVImg(rgbName.c_str(), pstData->rgbImg.data, pstData->rgbImg.size) != 0) {
            ROS_ERROR("save rgb image failed!");
        } else {
            ROS_INFO("save rgb image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), rgbName.c_str());
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string(m_serialno + "_PointCloud_"  + std::to_string(
                pstData->pointCloud.width) + "x" + std::to_string(pstData->pointCloud.height)
                                               + "_"  + std::to_string(m_pointCloudIndex++) + ".pcd"));
        if (savePointCloudWithPcdFormat(pointCloudName.c_str(), static_cast<float *>(pstData->pointCloud.data),
                                        pstData->pointCloud.size / sizeof(float)) != 0) {
            ROS_ERROR("save point cloud failed!");
        } else {
            ROS_INFO("save point cloud success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), pointCloudName.c_str());
        }
    }

    if (pstData->irImg.size > 0) {
        std::string irimgName(std::string(m_serialno + "_ir_" + std::to_string(pstData->irImg.width) + "x" +
                                          std::to_string(pstData->irImg.height) + "_" + std::to_string(m_irindex++) + ".yuv"));
        if (saveYUVImg(irimgName.c_str(), pstData->irImg.data, pstData->irImg.size) != 0) {
            ROS_ERROR("save ir image failed!");
        } else {
            ROS_INFO("save ir image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), irimgName.c_str());
        }
    }

    if (pstData->peakImg.size > 0) {
        std::string peakimgName(std::string(m_serialno + "_peak_") + std::to_string(
                                    pstData->peakImg.width) + "x" + std::to_string(pstData->peakImg.height)
                                + "_" + std::to_string(m_peakindex++) + ".yuv");
        if (saveYUVImg(peakimgName.c_str(), pstData->peakImg.data, pstData->peakImg.size) != 0) {
            ROS_ERROR("save peak image failed!");
        } else {
            ROS_INFO("save peak image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), peakimgName.c_str());
        }
    }

    return;
}

void Camera::saveMergeImage(const AS_SDK_MERGE_s *pstData)
{
    if (!m_save_merge_img) {
        return;
    }
    m_save_merge_img = false;
    if (pstData->depthImg.size > 0) {
        std::string depthimgName(std::string(m_serialno + "_depth_merge_") + std::to_string(
                                     pstData->depthImg.width) + "x" + std::to_string(pstData->depthImg.height)
                                 + "_" + std::to_string(m_depthindex++) + ".yuv");
        if (saveYUVImg(depthimgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            ROS_ERROR("save depth image failed!");
        } else {
            ROS_INFO("save depth image success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), depthimgName.c_str());
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string(m_serialno + "_PointCloud_merge_"  + std::to_string(
                pstData->pointCloud.width) + "x" + std::to_string(pstData->pointCloud.height)
                                               + "_"  + std::to_string(m_pointCloudIndex++) + ".pcd"));
        if (savePointCloudWithPcdFormat(pointCloudName.c_str(), static_cast<float *>(pstData->pointCloud.data),
                                        pstData->pointCloud.size / sizeof(float)) != 0) {
            ROS_ERROR("save point cloud failed!");
        } else {
            ROS_INFO("save point cloud success!");
            ROS_INFO("location: %s/%s", getcwd(nullptr, 0), pointCloudName.c_str());
        }
    }

    return;
}


std::string Camera::getColorFrameId()
{
    return m_frameIdInfo->getColorFrameId();
}

std::string Camera::getDepthFrameId()
{
    return m_frameIdInfo->getDepthFrameId();
}

std::string Camera::getDefaultFrameId()
{
    return m_frameIdInfo->getDefaultFrameId();
}

std::string Camera::getCamLinkFrameId()
{
    return m_frameIdInfo->getCamLinkFrameId();
}


/**
 * @file      CameraSrv.cpp
 * @brief     angstrong camera service.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/02/15
 * @version   1.0

 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <thread>
#include <sys/time.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <iostream>
#include <string>
#include <dirent.h>

#include "as_camera_sdk_api.h"
#include "common.h"
#include "CameraPublisher.h"
#include "CameraSrv.h"
#include "ros/ros.h"

CameraSrv::CameraSrv(ICameraStatus *cameraStatus, const std::string &filepath) : m_camera_status(cameraStatus)
{
    int ret = 0;
    ROS_INFO("Angstrong camera server");
    ret = AS_SDK_Init();
    if (ret != 0) {
        ROS_ERROR("sdk init failed");
    }
    char sdkVersion[64] = {0};
    ret = AS_SDK_GetSwVersion(sdkVersion, sizeof(sdkVersion));
    if (ret != 0) {
        ROS_ERROR("get sdk version failed");
    }
    ROS_INFO("Angstrong camera sdk version: %s", sdkVersion);

    m_configPath = filepath;
}

CameraSrv::~CameraSrv()
{
    ROS_INFO("Angstrong camera server exit");
    int ret = AS_SDK_Deinit();
    if (ret != 0) {
        ROS_ERROR("sdk deinit failed");
    }
}

int CameraSrv::start()
{
    AS_LISTENER_CALLBACK_S listener_callback;
    listener_callback.onAttached = onAttached;
    listener_callback.onDetached = onDetached;
    listener_callback.privateData = this;

    AS_SDK_StartListener(listener_callback, AS_LISTENNER_TYPE_USB, true);
    AS_SDK_StartListener(listener_callback, AS_LISTENNER_TYPE_NET, true);
    return 0;
}

void CameraSrv::stop()
{
    int ret = 0;
    int dev_idx = 0;

    ret = AS_SDK_StopListener();
    if (ret == 0) {
        ROS_INFO("stop listener monitor");
    }

    ROS_INFO("stop and close the camera");
    for (auto it = m_devsList.begin(); it != m_devsList.end(); ) {
        AS_CAM_PTR dev = (AS_CAM_PTR)(*it);
        ROS_INFO("close camera idx %d", dev_idx);

        ret = AS_SDK_StopStream(dev);
        if (ret < 0) {
            ROS_ERROR("stop stream, ret %d", ret);
        }
        ret = AS_SDK_CloseCamera(dev);
        if (ret < 0) {
            ROS_ERROR("close camera, ret %d", ret);
        }
        ret = AS_SDK_DestoryCamHandle(dev);
        if (ret == 0) {
            ROS_INFO("destory camera success");
        }
        m_devsList.erase(it++);
        dev_idx++;
    }
}

void CameraSrv::onAttached(AS_CAM_ATTR_S *attr, void *privateData)
{
    int ret = 0;
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    CamSvrStreamParam_s stream_param;
    stream_param.open = true;
    stream_param.start = true;
    stream_param.image_flag = 0;

    ROS_INFO("attached");
    std::lock_guard<std::timed_mutex> lock(server->m_mutex);

    bool exist = false;
    for (const auto &camera : server->m_devsList) {
        AS_CAM_ATTR_S attr_t;
        memset(&attr_t, 0, sizeof(AS_CAM_ATTR_S));
        ret = AS_SDK_GetCameraAttrs(camera, attr_t);
        if ((ret == 0) && ((attr->type == AS_CAMERA_ATTR_LNX_USB))) {
            if ((attr->attr.usbAttrs.bnum == attr_t.attr.usbAttrs.bnum)
                && (strcmp(attr->attr.usbAttrs.port_numbers, attr_t.attr.usbAttrs.port_numbers) == 0)) {
                ROS_WARN("this camera exist");
                exist = true;
                break;
            }
        } else if ((ret == 0) && (attr->type == AS_CAMERA_ATTR_NET)) {
            if (strcmp(attr->attr.netAttrs.ip_addr, attr_t.attr.netAttrs.ip_addr) == 0) {
                ROS_WARN("this camera exist");
                exist = true;
                break;
            }
        } else {
            ROS_ERROR("error camera attr");
            return;
        }
    }
    if (exist) {
        ROS_WARN("this device exist, ignore this event, attached end");
        return;
    }

    ROS_INFO("this is a new attach device, create and open it");
    AS_CAM_PTR newdev;
    ret = AS_SDK_CreateCamHandle(newdev, attr);
    if (ret == 0) {
        server->m_devsList.push_back(newdev);

        // get model type
        AS_SDK_CAM_MODEL_E cam_type;
        ret = AS_SDK_GetCameraModel(newdev, cam_type);
        ROS_INFO("get model type %d", cam_type);

        std::string file_path;
        if (server->getConfigFile(newdev, file_path, cam_type) != 0) {
            ROS_ERROR("cannot find config file");
            return;
        }

        server->m_camera_status->onCameraAttached(newdev, stream_param, cam_type);
        if (stream_param.open) {
            AS_CAM_Stream_Cb_s streamCallback;
            streamCallback.callback = server->onNewFrame;
            streamCallback.privateData = privateData;

            ret = AS_SDK_OpenCamera(newdev, file_path.c_str());
            if (ret < 0) {
                ROS_ERROR("open camera, ret: %d", ret);
                return;
            }
            server->m_camera_status->onCameraOpen(newdev);
            ret = AS_SDK_RegisterStreamCallback(newdev, &streamCallback);
            if (ret != 0) {
                ROS_ERROR("register stream callback failed");
            }

            if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                // Register merge stream callback
                AS_CAM_Merge_Cb_s mergeStreamCallback;
                mergeStreamCallback.callback = onNewMergeFrame;
                mergeStreamCallback.privateData = privateData;

                ret = AS_SDK_RegisterMergeFrameCallback(newdev, &mergeStreamCallback);
                if (ret != 0) {
                    ROS_ERROR("Register merge stream callback failed");
                    return;
                }
            }

            if (stream_param.start) {
                ret = AS_SDK_StartStream(newdev, stream_param.image_flag);
                if (ret < 0) {
                    ROS_ERROR("start stream, ret: %d", ret);
                    return;
                }
                server->m_camera_status->onCameraStart(newdev);
            }
        }
    }
    ROS_INFO("attached end");
}

void CameraSrv::onDetached(AS_CAM_ATTR_S *attr, void *privateData)
{
    int ret = 0;
    ROS_INFO("detached");
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    std::lock_guard<std::timed_mutex> lock(server->m_mutex);
    for (auto it = server->m_devsList.begin(); it != server->m_devsList.end(); it++) {
        AS_CAM_PTR dev = (AS_CAM_PTR)(*it);
        AS_CAM_ATTR_S t_attr;
        ret = AS_SDK_GetCameraAttrs(dev, t_attr);
        if (ret < 0) {
            ROS_ERROR("get camera attribute failed");
            return;
        }

        if (t_attr.type == AS_CAMERA_ATTR_LNX_USB) {
            if ((t_attr.attr.usbAttrs.bnum == attr->attr.usbAttrs.bnum)
                && (t_attr.attr.usbAttrs.dnum == attr->attr.usbAttrs.dnum)) {
                ROS_INFO("close and delete it from the list");
                server->m_camera_status->onCameraStop(dev);
                ret = AS_SDK_StopStream(dev, 0);
                if (ret != 0) {
                    ROS_INFO("stop stream failed");
                } else {
                    ROS_INFO("stop stream success");
                }
                server->m_camera_status->onCameraClose(dev);
                ret = AS_SDK_CloseCamera(dev);
                if (ret != 0) {
                    ROS_INFO("close camera failed");
                } else {
                    ROS_INFO("close camera success");
                }
                server->m_camera_status->onCameraDetached(dev);
                ret = AS_SDK_DestoryCamHandle(dev);
                if (ret == 0) {
                    ROS_INFO("destory camera success");
                }
                server->m_devsList.erase(it);
                break;
            }
        } else if (t_attr.type == AS_CAMERA_ATTR_NET) {
            if (strcmp(t_attr.attr.netAttrs.ip_addr, attr->attr.netAttrs.ip_addr) == 0) {
                ROS_INFO("close and delete it from the list");
                server->m_camera_status->onCameraStop(dev);
                ret = AS_SDK_StopStream(dev, 0);
                if (ret != 0) {
                    ROS_INFO("stop stream failed");
                } else {
                    ROS_INFO("stop stream success");
                }
                server->m_camera_status->onCameraClose(dev);
                ret = AS_SDK_CloseCamera(dev);
                if (ret != 0) {
                    ROS_INFO("close camera failed");
                } else {
                    ROS_INFO("close camera success");
                }
                server->m_camera_status->onCameraDetached(dev);
                ret = AS_SDK_DestoryCamHandle(dev);
                if (ret == 0) {
                    ROS_INFO("destory camera success");
                }
                server->m_devsList.erase(it);
                break;
            }
        } else {
            ROS_ERROR("camera attr type error");
            return;
        }

    }
    ROS_INFO("detached end");
}

void CameraSrv::onNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData, void *privateData)
{
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    server->m_camera_status->onCameraNewFrame(pCamera, pstData);
}

void CameraSrv::onNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData, void *privateData)
{
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    server->m_camera_status->onCameraNewMergeFrame(pCamera, pstData);
}

int CameraSrv::getConfigFile(AS_CAM_PTR pCamera, std::string &configfile, AS_SDK_CAM_MODEL_E cam_type)
{
    int ret = 0;
    std::string name_key;
    switch (cam_type) {
    case AS_SDK_CAM_MODEL_KONDYOR:
    case AS_SDK_CAM_MODEL_KONDYOR_NET:
        name_key = "kondyor_";
        break;
    case AS_SDK_CAM_MODEL_NUWA_XB40:
    case AS_SDK_CAM_MODEL_NUWA_X100:
    case AS_SDK_CAM_MODEL_NUWA_HP60:
    case AS_SDK_CAM_MODEL_NUWA_HP60V:
        name_key = "nuwa_";
        break;
    case AS_SDK_CAM_MODEL_KUNLUN_A:
    case AS_SDK_CAM_MODEL_KUNLUN_C:
        name_key = "kunlun_";
        break;
    case AS_SDK_CAM_MODEL_HP60C:
        name_key = "hp60c_";
        break;
    case AS_SDK_CAM_MODEL_HP60CN:
        name_key = "hp60cn_";
        break;
    case AS_SDK_CAM_MODEL_VEGA:
        name_key = "vega_";
        break;
    case AS_SDK_CAM_MODEL_CHANGJIANG_B:
        name_key = "changjiangB_";
        break;
    case AS_SDK_CAM_MODEL_TANGGULA:
        name_key = "tanggula_";
        break;
    case AS_SDK_CAM_MODEL_TANGGULA_A:
        name_key = "tanggulaA_";
        break;
    case AS_SDK_CAM_MODEL_TAISHAN:
        name_key = "taishan_";
        break;
    case AS_SDK_CAM_MODEL_TANGGULA_B:
        name_key = "tanggulaB_";
        break;
    default:
        ROS_ERROR("cam type error");
        return -1;
        break;
    }

    // get json
    std::vector<std::string> files;
    scanDir(m_configPath, files);
    ret = -1;
    for (auto it = files.begin(); it != files.end(); it++) {
        std::string filename = (*it).substr((*it).find_last_of("/"));
        if (filename.find(name_key) < filename.size()) {
            ROS_INFO("get file: %s", it->c_str());
            configfile = (*it);
            ret = 0;
            break;
        }
    }

    if (ret != 0) {
        ROS_ERROR("cannot find config file");
        return -1;
    }

    return 0;
}

int CameraSrv::scanDir(const std::string &dir, std::vector<std::string> &file)
{
    int ret = 0;
    DIR *directory;
    struct dirent *ent;
    if (!(directory = opendir(dir.c_str()))) {
        ROS_ERROR("can't not open dir: %s", dir.c_str());
        return -1;
    }
    while ((ent = readdir(directory)) != nullptr) {
        if (strncmp(ent->d_name, ".", 1) == 0) {
            continue;
        }
        if (ent->d_type == DT_REG) {
            std::string filepath(dir + "/" + ent->d_name);
            file.push_back(filepath);
        }
        if (ent->d_type == DT_DIR) {
            std::string childpath;
            childpath.append(dir);
            childpath.append("/");
            childpath.append(ent->d_name);
            scanDir(childpath, file);
        }
    }
    delete ent;
    closedir(directory);
    return ret;
}


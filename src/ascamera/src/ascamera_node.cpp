/**
 * @file      ascamera_node.cpp
 * @brief     angstrong ros camera publisher node.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */

#include <errno.h>
#include <thread>
#include <malloc.h>
#include <string.h>
#include <iostream>
#include <memory>

#include "ascamera_node.h"
#include "CameraPublisher.h"
#include "ros/ros.h"
#include "LogRedirectBuffer.h"

int main(int argc, char *argv[])
{
    int ret = 0;
    ROS_INFO("Hello, angstrong camera node");
    ros::init(argc, argv, "ascamera");

    auto buf = std::make_shared<LogRedirectBuffer>();
    std::cout.rdbuf(buf.get());
    std::cerr.rdbuf(buf.get());

    ros::NodeHandle nh("~");

    auto cameraPublisher = std::make_shared<CameraPublisher>(nh);

    ret = cameraPublisher->start();
    if (ret != 0) {
        ROS_ERROR("start camera publisher failed");
        return -1;
    }

    std::thread cmdThread([&] {
        fd_set readFdSet;
        struct timeval timeout;
        while (ros::ok())
        {
            FD_ZERO(&readFdSet);
            FD_SET(fileno(stdin), &readFdSet);
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            select(fileno(stdin) + 1, &readFdSet, nullptr, nullptr, &timeout);
            if (FD_ISSET(fileno(stdin), &readFdSet)) {
                char ch = getchar();
                if (ch == 's') {
                    cameraPublisher->saveImage();
                } else if (ch == 'q') {
                    ros::shutdown();
                    break;
                } else if (ch == 'f') {
                    cameraPublisher->logFps(!cameraPublisher->getLogFps());
                } else if (ch == 'l') {
                    cameraPublisher->logCfgParameter();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    ros::spin();
    if (cmdThread.joinable()) {
        cmdThread.join();
    }

    cameraPublisher->stop();

    return 0;
}

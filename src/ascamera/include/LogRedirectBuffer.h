#pragma once

#include <string>
#include <streambuf>
#include "ros/ros.h"

class LogRedirectBuffer : public std::streambuf
{
public:
    explicit LogRedirectBuffer()
    {
        buffer_ = new char[BUFFER_SIZE];
        setp(buffer_, buffer_ + BUFFER_SIZE);
    }

    ~LogRedirectBuffer() override
    {
        sync();
        delete[] buffer_;
    }

protected:
    int sync() override
    {
        if (pbase() != pptr()) {
            std::string msg(pbase(), pptr() - pbase());
            msg.erase(std::remove(msg.begin(), msg.end(), '\n'), msg.end());
            msg.erase(std::remove(msg.begin(), msg.end(), '\r'), msg.end());
            ROS_INFO_STREAM(msg);
            setp(buffer_, buffer_ + BUFFER_SIZE);
        }
        return 0;
    }

    int overflow(int c) override
    {
        sync();
        if (c != EOF) {
            *pptr() = c;
            pbump(1);
        }
        return c;
    }

private:
    static const int BUFFER_SIZE = 1024;
    char *buffer_;
};
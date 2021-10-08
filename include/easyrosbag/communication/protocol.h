#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <iostream>
#include <vector>
#include "utils/point_cloud.h"
#include "utils/common.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

namespace protocol
{
    class Protocol
    {
    public:
        Protocol();
        ~Protocol();

        // pcd message
        void decodeProtocolJson(std::string &jsonStr, std::vector<PointType> &pcd);
        void codeProtocolJson(std::vector<PointType> &pcd, std::string &jsonStr);

        // imu message
        void codeProtocolJson(easyrosbag::ImuOutput &imu, std::string &jsonStr);
        void decodeProtocolJson(std::string &jsonStr, easyrosbag::ImuOutput &imu);

        // // pose message
        void decodeProtocolJson(std::string &jsonStr, easyrosbag::Pose &pose);
        void codeProtocolJson(easyrosbag::Pose &pose, std::string &jsonStr);

        // image message

        // wheel odomery message

        // GPS message

    };
} // namespace protocol
#endif

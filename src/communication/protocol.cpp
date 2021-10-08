#include "string.h"
#include "protocol.h"
#include "stdio.h"
#include <iostream>
#include <math.h>

namespace protocol
{
    Protocol::Protocol()
    {
    }
    Protocol::~Protocol()
    {
    }

    void Protocol::codeProtocolJson(std::vector<PointType> &pcd, std::string &jsonStr){
        rapidjson::Document doc;
        doc.SetObject();
        rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
        // type
        doc.AddMember("type", "PCD", allocator);
        // count
        doc.AddMember("count", 0, allocator);
        // discrible
        rapidjson::Value array_sub(rapidjson::kArrayType);
        array_sub.PushBack(1, allocator);
        array_sub.PushBack(pcd.size(), allocator);
        doc.AddMember("discrible", array_sub, allocator);
        // data
        uint32_t i;
        rapidjson::Value array_data(rapidjson::kArrayType);
        // printf("encode laser cornerless count: %d\n", pcd.size());

        for (i = 0; i < pcd.size(); i++)
        {
            rapidjson::Value array_point(rapidjson::kArrayType);

            {
                array_point.PushBack(pcd[i].x, allocator);
                array_point.PushBack(pcd[i].y, allocator);
                array_point.PushBack(pcd[i].z, allocator);
                array_data.PushBack(array_point, allocator);
            }
        }
        doc.AddMember("data", array_data, allocator);
        rapidjson::StringBuffer s;
        rapidjson::Writer<rapidjson::StringBuffer> writer(s);
        doc.Accept(writer);
        jsonStr = s.GetString();
    }

    void Protocol::decodeProtocolJson(std::string &jsonStr, std::vector<PointType> &pcd){
        rapidjson::Document doc;
        doc.Parse(jsonStr.c_str());
        std::string type = doc["type"].GetString();
        if (!type.compare("PCD"))
        {
            rapidjson::Value &discible = doc["discrible"];
            rapidjson::Value &data = doc["data"];
            long long int count = doc["count"].GetInt64();

            // pcd.height = discible[0].GetUint();
            // pcd.width = discible[1].GetUint();
            // pcd.is_dense = discible[2].GetBool();
            if (data.IsArray())
            {
                for (int index = 0; index < data.Size(); index++)
                {
                    float x_coord, y_coord, z_coord;
                    uint8_t intensity, ring;

                    x_coord = data[index][0].GetFloat();
                    y_coord = data[index][1].GetFloat();
                    z_coord = data[index][2].GetFloat();

                    PointType point;
                    point.x = x_coord;
                    point.y = y_coord;
                    point.z = z_coord;
                    pcd.push_back(point);
                }
            }
        }
        else
        {
            // printf("type error!\r\n");
            // std::cout << "[json type]:" << type << std::endl;
        }
    }

    void Protocol::codeProtocolJson(easyrosbag::ImuOutput &imu, std::string &jsonStr)
    {
        rapidjson::Document doc;
        doc.SetObject();
        rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
        // type
        doc.AddMember("type", "IMU", allocator);
        // data
        rapidjson::Value array_data(rapidjson::kArrayType);
        array_data.PushBack(imu.timestamp, allocator);

        array_data.PushBack(imu.acc.x(), allocator);
        array_data.PushBack(imu.acc.y(), allocator);
        array_data.PushBack(imu.acc.z(), allocator);

        array_data.PushBack(imu.gyr.x(), allocator);
        array_data.PushBack(imu.gyr.y(), allocator);
        array_data.PushBack(imu.gyr.z(), allocator);

        doc.AddMember("data", array_data, allocator);
        rapidjson::StringBuffer s;
        rapidjson::Writer<rapidjson::StringBuffer> writer(s);
        doc.Accept(writer);
        jsonStr = s.GetString();
    }

    void Protocol::decodeProtocolJson(std::string &jsonStr, easyrosbag::ImuOutput &imu)
    {
        rapidjson::Document doc;
        doc.Parse(jsonStr.c_str());
        std::string type = doc["type"].GetString();
        if(!type.compare("IMU"))
        {
            rapidjson::Value &data = doc["data"];
            imu.timestamp = data[0].GetDouble();
            imu.acc.x() = data[1].GetDouble();
            imu.acc.y() = data[2].GetDouble();
            imu.acc.z() = data[3].GetDouble();
            imu.gyr.x() = data[4].GetDouble();
            imu.gyr.y() = data[5].GetDouble();
            imu.gyr.z() = data[6].GetDouble();
        }
    }

    void Protocol::decodeProtocolJson(std::string &jsonStr, easyrosbag::Pose &pose)
    {
        rapidjson::Document doc;
        doc.SetObject();
        rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
        // type
        doc.AddMember("type", "POSE", allocator);
        // data
        rapidjson::Value array_data(rapidjson::kArrayType);
        array_data.PushBack(pose.timestamp, allocator);

        array_data.PushBack(pose.pose.x(), allocator);
        array_data.PushBack(pose.pose.y(), allocator);
        array_data.PushBack(pose.pose.z(), allocator);

        array_data.PushBack(pose.quat.w(), allocator);
        array_data.PushBack(pose.quat.x(), allocator);
        array_data.PushBack(pose.quat.y(), allocator);
        array_data.PushBack(pose.quat.z(), allocator);

        doc.AddMember("data", array_data, allocator);
        rapidjson::StringBuffer s;
        rapidjson::Writer<rapidjson::StringBuffer> writer(s);
        doc.Accept(writer);
        jsonStr = s.GetString();

    }
    
    void Protocol::codeProtocolJson(easyrosbag::Pose &pose, std::string &jsonStr)
    {
        rapidjson::Document doc;
        doc.Parse(jsonStr.c_str());
        std::string type = doc["type"].GetString();
        if(!type.compare("POSE"))
        {
            rapidjson::Value &data = doc["data"];
            pose.timestamp = data[0].GetDouble();
            pose.pose.x() = data[1].GetDouble();
            pose.pose.y() = data[2].GetDouble();
            pose.pose.z() = data[3].GetDouble();
            pose.quat.w() = data[4].GetDouble();
            pose.quat.x() = data[4].GetDouble();
            pose.quat.y() = data[5].GetDouble();
            pose.quat.z() = data[6].GetDouble();
        }
    }

} // namespace protocol
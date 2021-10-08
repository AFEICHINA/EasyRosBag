#include <iostream>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "rosbag/bag.h"
#include <rosbag/view.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/point_field_conversion.h"

#include "sensor_msgs/Imu.h"

struct PointXYZIRT {
    float x;
    float y;
    float z;
    double intensity;
    uint8_t ring;
    double timestamp;
};

typedef PointXYZIRT PointType;

size_t TimmoLidarReader(sensor_msgs::PointCloud2::ConstPtr pcd_msg, std::vector<PointType> &pcd){
    size_t pt_size = pcd_msg->width * pcd_msg->height;
    // // Get the x/y/z field offsets
    int x_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "x");
    int y_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "y");
    int z_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "z");
    int inten_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "intensity");
    int ring_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "ring");
    int timestamp_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "timestamp");  // from alive-capture driver

    bool bFindTimestamp = false;
    bool bFindRing = false;

    if (timestamp_idx >= 0) {
        bFindTimestamp = true;
    }

    if (ring_idx >= 0) {
        bFindRing = true;
        // std::cout << "find ring field in sensor_msgs::PointCloud!" <<
        // std::endl;
    }

    int x_offset = pcd_msg->fields[x_idx].offset;
    int y_offset = pcd_msg->fields[y_idx].offset;
    int z_offset = pcd_msg->fields[z_idx].offset;
    int inten_offset = pcd_msg->fields[inten_idx].offset;
    int ring_offset = pcd_msg->fields[ring_idx].offset;
    int timestamp_offset = pcd_msg->fields[timestamp_idx].offset;

    uint8_t x_datatype = pcd_msg->fields[x_idx].datatype;
    uint8_t y_datatype = pcd_msg->fields[y_idx].datatype;
    uint8_t z_datatype = pcd_msg->fields[z_idx].datatype;
    uint8_t inten_datatype = pcd_msg->fields[inten_idx].datatype;
    uint8_t ring_datatype = pcd_msg->fields[ring_idx].datatype;
    uint8_t timestamp_datatype = pcd_msg->fields[timestamp_idx].datatype;

    // Copy the data points
    size_t validCount = 0;
    for (size_t cp = 0; cp < pt_size; ++cp) {
        // Copy x/y/z/intensity
        PointType pt;
        pt.x = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + x_offset], x_datatype);
        pt.y = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + y_offset], y_datatype);
        pt.z = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + z_offset], z_datatype);

        if (inten_idx >= 0)
            pt.intensity = sensor_msgs::readPointCloud2BufferValue<uint8_t>(&pcd_msg->data[cp * pcd_msg->point_step +inten_offset], inten_datatype);

        if (ring_idx >= 0){
            uint8_t pt_ring = sensor_msgs::readPointCloud2BufferValue<uint8_t>(&pcd_msg->data[cp * pcd_msg->point_step + ring_offset], ring_datatype);
            pt.ring = pt_ring;
        }

        if (timestamp_idx >= 0) {
            // alive capture
            double pt_time = sensor_msgs::readPointCloud2BufferValue<double>(&pcd_msg->data[cp * pcd_msg->point_step +timestamp_offset], timestamp_datatype);
            pt.timestamp = pt_time;
        }
        printf("point x:%f y:%f z:%f intensity: %lf ring:%d timestamp:%lf\n", pt.x, pt.y, pt.z, pt.intensity, pt.ring, pt.timestamp);
        pcd.push_back(pt);
        validCount++;
    }
    return validCount;
}


int main(int argc, char** argv) {
    std::string path;
    std::string topic;
    std::vector<std::string> topics;
    if (argc < 3){
        // path = "/media/afei/Samsung_T3/dataset/timmo/timmo-fix.bag";
        // topics.push_back(std::string("/lidar_pointcloud"));
        // topics.push_back(std::string("/BMI088"));
        printf("Usages:./read_rosbag_data rosbag_path topic[1] topic[2]\n");
        return 0;
    }
    else if(argc < 4){
        path = argv[1];
        topic = argv[2];
        topics.push_back(topic);
    }
    else if(argc < 5){
        path = argv[1];
        topics.push_back(argv[2]);
        topics.push_back(argv[3]);
    }

    std::cout << "Read Rosbag Path: " << path << std::endl;

    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        std::cout << "LOADING BAG FAILED: " << e.what() << std::endl;
        return 0;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::vector<const rosbag::ConnectionInfo*> info;
    info = view.getConnections();

    for(auto i : info)
        std::cout << i->topic << std::endl;

    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::ConstPtr pcd_msg = m.instantiate<sensor_msgs::PointCloud2>();
        sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();

        std::vector<PointType> pcd;
        if (pcd_msg != NULL) {
            TimmoLidarReader(pcd_msg, pcd);
            printf("\n");
            printf("pcd size: %zu\n", pcd.size());
        }

        if(imu_msg != NULL) {
            std::cout << "acc: " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << "\n"
                      << "gyr: " << imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << std::endl;
        }
    }

    bag.close();
    return 0;
}
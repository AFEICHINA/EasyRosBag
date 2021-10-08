#include "easyrosbag/easy_rosbag.hpp"

using namespace easyrosbag;

ParameterReader::Ptr mConfig;
communication::MqttClient mqc("rosbag_server");

size_t TimmoLidarReader(sensor_msgs::PointCloud2::ConstPtr pcd_msg,
                        std::vector<PointType> &pcd) {
    size_t pt_size = pcd_msg->width * pcd_msg->height;
    // // Get the x/y/z field offsets
    int x_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "x");
    int y_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "y");
    int z_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "z");
    int inten_idx =
        sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "intensity");
    int ring_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "ring");
    int timestamp_idx = sensor_msgs::getPointCloud2FieldIndex(
        *pcd_msg, "timestamp");  // from alive-capture driver

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
        pt.x = sensor_msgs::readPointCloud2BufferValue<float>(
            &pcd_msg->data[cp * pcd_msg->point_step + x_offset], x_datatype);
        pt.y = sensor_msgs::readPointCloud2BufferValue<float>(
            &pcd_msg->data[cp * pcd_msg->point_step + y_offset], y_datatype);
        pt.z = sensor_msgs::readPointCloud2BufferValue<float>(
            &pcd_msg->data[cp * pcd_msg->point_step + z_offset], z_datatype);

        if (pt.x == 0 && pt.y == 0 && pt.y == 0)
            continue;
        if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
            continue;

        if (inten_idx >= 0)
            pt.intensity = sensor_msgs::readPointCloud2BufferValue<uint8_t>(
                &pcd_msg->data[cp * pcd_msg->point_step + inten_offset],
                inten_datatype);

        if (ring_idx >= 0) {
            uint8_t pt_ring = sensor_msgs::readPointCloud2BufferValue<uint8_t>(
                &pcd_msg->data[cp * pcd_msg->point_step + ring_offset],
                ring_datatype);
            pt.ring = pt_ring;
        }

        if (timestamp_idx >= 0) {
            // alive capture
            double pt_time = sensor_msgs::readPointCloud2BufferValue<double>(
                &pcd_msg->data[cp * pcd_msg->point_step + timestamp_offset],
                timestamp_datatype);
            pt.timestamp = pt_time;
        }
        // printf("point x:%f y:%f z:%f intensity: %lf ring:%d timestamp:%lf\n",
        // pt.x, pt.y, pt.z, pt.intensity, pt.ring, pt.timestamp);
        pcd.push_back(pt);
        validCount++;
    }
    return validCount;
}

void pub_pcd(sensor_msgs::PointCloud2::ConstPtr pcd_msg, std::string topic) {
    std::vector<PointType> pcd;
    std::vector<PointType> pcd_tmp;
    TimmoLidarReader(pcd_msg, pcd);
    for(int i = 0;i<pcd.size();i=i+2){
        pcd_tmp.push_back(pcd[i]);
    }

    std::string playload_content;
    protocol::Protocol prtc;
    //这里点有数量限制,不然会重连,需要修改emqx conf
    prtc.codeProtocolJson(pcd, playload_content);
    communication::PubMessge pcdmsg(topic, (void *)playload_content.c_str(),
                                    strlen(playload_content.c_str()), false, 0);
    mqc.mqcPublish(pcdmsg);
}

void pub_imu(sensor_msgs::Imu::ConstPtr imu_msg, std::string topic) { 
    ImuOutput imu;
    imu.timestamp = imu_msg->header.stamp.toSec();
    imu.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    imu.gyr = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    std::string playload_content;
    protocol::Protocol prtc;
    prtc.codeProtocolJson(imu, playload_content);
    communication::PubMessge imumsg(topic, (void *)playload_content.c_str(),
                                    strlen(playload_content.c_str()), false, 0);
    mqc.mqcPublish(imumsg);
}

void pub_pose(nav_msgs::Odometry::ConstPtr pose_msg, std::string topic) {
    Pose pose;
    pose.timestamp = pose_msg->header.stamp.toSec();
    pose.pose = Eigen::Vector3d(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    pose.quat = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, 
                                    pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
    std::string playload_content;
    protocol::Protocol prtc;
    prtc.codeProtocolJson(pose, playload_content);
    communication::PubMessge posemsg(topic, (void *)playload_content.c_str(),
                                    strlen(playload_content.c_str()), false, 0);
    mqc.mqcPublish(posemsg);
}


void usage() { std::cout << "Usage: ./rosbag_play YourBagPath  " 
            << std::endl << "or     ./rosbag_play YourBagPath YourConfigPath" << std::endl; }

int main(int argc, char **argv) {
    std::string bagPath;
    std::string strParam;

    if (argc < 2) {
        usage();
        return 0;
    }
    else if(argc == 2){
        bagPath = argv[1];
        strParam = "../config/parameters.txt";
    }else{
        bagPath = argv[1];
        strParam = argv[2];
    }

    mConfig = ParameterReader::create(strParam);
    
    std::string strHostIP = mConfig->getData("board_ip");
    std::string strPort = mConfig->getData("port");
    int port = atoi(strPort.c_str());

    communication::MqcCfg mqc_cfg(strHostIP.data(), port, 60);
    mqc.mqcInit(mqc_cfg);
    mqc.mqcStart();
    printf("mqtt client start success!\n");

    rosbag::Bag bag;
    try {
        bag.open(bagPath, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        std::cout << "LOADING BAG FAILED: " << e.what() << std::endl;
        return 0;
    }

    // get topics
    std::vector<std::string> topics;
    {
        rosbag::View view(bag);
        std::vector<const rosbag::ConnectionInfo *> info;
        info = view.getConnections();

        printf("Get topic size: %ld\n", info.size());

        for (auto i : info) {
            std::cout << i->topic << std::endl;
            topics.push_back(i->topic);
        }
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const m : view) {
        std::cout << "[RUNNING]  Bag Time:" << m.getTime() << std::endl;
        sensor_msgs::PointCloud2::ConstPtr pcd_msg = m.instantiate<sensor_msgs::PointCloud2>(); 
        sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        nav_msgs::Odometry::ConstPtr pose_msg = m.instantiate<nav_msgs::Odometry>();

        std::string topic = m.getTopic();
        if (pcd_msg != NULL) {
            pub_pcd(pcd_msg, topic);
        }

        if(imu_msg != NULL) {
            pub_imu(imu_msg, topic);
        }

        if(pose_msg != NULL) {
            pub_pose(pose_msg, topic);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    bag.close();
    mqc.mqcExit();
    return 0;
}

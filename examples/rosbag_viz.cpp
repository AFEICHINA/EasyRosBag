#include "easyrosbag/easy_rosbag.hpp"

using namespace easyrosbag;

ParameterReader::Ptr mConfig;
communication::MqttClient mqcSub("rosbag_viz");
std::vector<PointType> cur_pcd_;
RosBagViewer::Ptr view = RosBagViewer::create();

void pcdCallBack(std::vector<PointType> &pcd)
{  
    if(pcd.size() > 0){
        printf("\n");
        printf("get lidar num:%zu\n", pcd.size());
        cur_pcd_.clear();
        cur_pcd_ = pcd;
        view->setScan(cur_pcd_);
    }
}

void imuCallBack(ImuOutput &imu)
{
    if(imu.timestamp > 0)
    {
        std::cout << "acc:" << imu.acc.transpose() << " gyr:" << imu.gyr.transpose() << std::endl;
    }
}

void poseCallBack(Pose &pose)
{
    if(pose.timestamp > 0)
    {

    }
}

void usage(){ 
    std::cout << "-------------Usage---------------" << std::endl;
    std::cout << "Usage:./rosbag_viz" << std::endl 
              << "or   :./rosbag_viz ./config/parameters.txt" << std::endl; 
    std::cout << "---------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
    usage();

    std::string strParam;
    if(argc < 2){
        strParam = "../config/parameters.txt";
    }else{
        strParam = argv[1];
    }

    mConfig = ParameterReader::create(strParam);

    std::string strHostIP = mConfig->getData("board_ip");
    // printf("set board_ip:%s ", strHostIP.data());
    std::string strPort = mConfig->getData("port");
    int port = atoi(strPort.c_str());
    // printf("port:%d\n", port);

    communication::MqcCfg mqc_cfg(strHostIP.data(), port, 60);
    mqcSub.mqcInit(mqc_cfg);
    mqcSub.mqcStart();

    std::string topicPCD = mConfig->getData("lidar_topic");
    std::string topicIMU = mConfig->getData("imu_topic");
    std::string topicPose = mConfig->getData("pose_topic");
    std::string topicImage = mConfig->getData("image_topic");

    if(topicPCD.data()){
        mqcSub.mqcRegistPCDCallback(pcdCallBack);
        mqcSub.mqcSubscrible(topicPCD);
    }
        
    if(topicIMU.data()){
        mqcSub.mqcRegistIMUCallback(imuCallBack);
        mqcSub.mqcSubscrible(topicIMU);
    }
        
    if(topicPose.data()){
        mqcSub.mqcRegistPoseCallback(poseCallBack);
        mqcSub.mqcSubscrible(topicPose);
    }
        
    if(topicImage.data()){
        // mqcSub.mqcRegistImageCallback(imageCallBack);
        mqcSub.mqcSubscrible(topicImage);
    }
        
    while (1)
    {
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }

    void mqcExit();
    return 0;
}
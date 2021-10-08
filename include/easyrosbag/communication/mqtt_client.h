#ifndef __MQTT_CLIENT_H__
#define __MQTT_CLIENT_H__
#include "mqc_cfg.h"
#include <mutex>
#include "mosquittopp.h"
#include <vector>
#include <thread>
#include <functional>
#include "utils/point_cloud.h"
#include "utils/common.h"

namespace communication
{
    class MqttClient : public mosqpp::mosquittopp
    {
    public:
        MqttClient(const char *mqcID);
        ~MqttClient();
        bool mqcInit(const MqcCfg &mqcCfg);
        void mqcStart();
        void mqcExit();
        int mqcPublish(const PubMessge &pubMessge);
        int mqcSubscrible(const std::string subTopic);
        
        void mqcRegistCallback(std::function<void(std::string&)> msg_callback);
        void mqcRegistPCDCallback(std::function<void(std::vector<PointType> &)> pcd_callback);
        void mqcRegistIMUCallback(std::function<void(easyrosbag::ImuOutput &)> imu_callback);
        // void mqcRegistImageCallback(std::function<void(easyrosbag::Image &)> image_callback);
        void mqcRegistPoseCallback(std::function<void(easyrosbag::Pose &)> pose_callback);
        uint64_t getCurrenTimeMs();

    private:
        void on_connect(int rc);
        void on_message(const struct mosquitto_message *message);

        bool loop_flag_;
        bool connect_flag_;
        void mqcMainLoop();
        void messagePubLoop();
        void messageDecodeLoop();

        std::shared_ptr<std::thread> mqc_main_loop_thr_;
        std::shared_ptr<std::thread> pub_messges_thr_;
        std::shared_ptr<std::thread> decode_messges_thr_;
        std::mutex pub_messge_mtx_, decode_msg_mtx_;
        std::vector<struct PubMessge> pub_messges_vec_;
        std::vector<std::string> recv_messges_vec_;
        std::vector<std::string> sub_topic_vec_;

        std::function<void(std::string&)> msg_callback_;
        std::function<void(std::vector<PointType> &)> pcd_callback_;
        std::function<void(easyrosbag::ImuOutput &)> imu_callback_;
        std::function<void(easyrosbag::Pose &)> pose_callback_;
        // std::function<void(easyrosbag::Image &)> image_callback_;
    };
} // namespace alive
#endif

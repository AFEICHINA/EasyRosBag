#include <mqtt_client.h>
#include <stdio.h>
#include <sys/time.h>
#include <set>
#include <algorithm>
#include "string.h"
#include <iostream>
#include "protocol.h"

namespace communication
{
    MqttClient::MqttClient(const char *mqcID) : mosquittopp(mqcID)
    {
        mosqpp::lib_init();
        loop_flag_ = true;
        connect_flag_ = false;
        mqc_main_loop_thr_ = nullptr;
        pub_messges_thr_ = nullptr;
        decode_messges_thr_ = nullptr;
    }

    MqttClient::~MqttClient()
    {
        mqcExit();
    }

    bool MqttClient::mqcInit(const MqcCfg &mqcCfg)
    {
        this->connect(mqcCfg.ip.c_str(), mqcCfg.port, mqcCfg.keepalive);
        printf(" -- ip:%s ,port:%d ,keepalive:%d.\r\n", mqcCfg.ip.c_str(), mqcCfg.port, mqcCfg.keepalive);
        return true;
    }

    void MqttClient::mqcStart()
    {
        this->mqc_main_loop_thr_.reset(new std::thread(std::bind(&MqttClient::mqcMainLoop, this)));
        // this->pub_messges_thr_.reset(new std::thread(std::bind(&MqttClient::messagePubLoop, this)));
        // this->decode_messges_thr_.reset(new std::thread(std::bind(&MqttClient::messageDecodeLoop, this)));
    }

    void MqttClient::mqcExit()
    {
        connect_flag_ = false;
        loop_flag_ = false;
        if (mqc_main_loop_thr_ && mqc_main_loop_thr_->joinable())
            mqc_main_loop_thr_->join();
        if (pub_messges_thr_ && pub_messges_thr_->joinable())
            pub_messges_thr_->join();
        if (decode_messges_thr_ && decode_messges_thr_->joinable())
            decode_messges_thr_->join();
        pub_messges_vec_.clear();
        recv_messges_vec_.clear();
        mosqpp::lib_cleanup();
    }

    int MqttClient::mqcPublish(const PubMessge &pubMessge)
    {
        // pub_messge_mtx_.lock();
        // pub_messges_vec_.push_back(pubMessge);
        // pub_messge_mtx_.unlock();
        if (connect_flag_)
        {
            publish(NULL, pubMessge.topic.c_str(), pubMessge.payloadlen, pubMessge.payload, pubMessge.qos, pubMessge.retain);
        }
        return 0;
    }

    int MqttClient::mqcSubscrible(const std::string subTopic)
    {
        sub_topic_vec_.push_back(subTopic);
        //去掉重复订阅的主题
        std::set<std::string> s(sub_topic_vec_.begin(), sub_topic_vec_.end());
        sub_topic_vec_.assign(s.begin(), s.end());

        for (auto iter = sub_topic_vec_.cbegin(); iter != sub_topic_vec_.cend(); iter++)
        {
            subscribe(NULL, (*iter).c_str());
        }
        return 0;
    }

    void MqttClient::mqcRegistCallback(std::function<void(std::string &)> msg_callback)
    {
        if (msg_callback)
            msg_callback_ = msg_callback;
    }

    void MqttClient::mqcRegistPCDCallback(std::function<void(std::vector<PointType> &)> pcd_callback)
    {
        if (pcd_callback)
            pcd_callback_ = pcd_callback;
    }

    void MqttClient::mqcRegistIMUCallback(std::function<void(easyrosbag::ImuOutput &)> imu_callback)
    {
        if (imu_callback)
            imu_callback_ = imu_callback;
    }

    void MqttClient::mqcRegistPoseCallback(std::function<void(easyrosbag::Pose &)> pose_callback)
    {
        if (pose_callback)
            pose_callback_ = pose_callback;
    }

    uint64_t MqttClient::getCurrenTimeMs()
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec * 1000 + tv.tv_usec / 1000;
    }

    void MqttClient::mqcMainLoop()
    {
        int rc;
        while (loop_flag_)
        {
            rc = loop();
            if (rc)
            {
                connect_flag_ = false;
                reconnect_async();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
    }

    void MqttClient::messagePubLoop()
    {
        while (loop_flag_)
        {
            if (connect_flag_)
            {
                //  printf("time: %ld  ,topic:%s ,payloadlen:%d.\r\n", getCurrenTimeMs(), packet.topic.c_str(), packet.payloadlen);
                if (pub_messges_vec_.size() > 0)
                {
                    pub_messge_mtx_.lock();
                    struct PubMessge packet = pub_messges_vec_.front();
                    pub_messges_vec_.pop_back();
                    pub_messge_mtx_.unlock();
                    publish(NULL, packet.topic.c_str(), packet.payloadlen, packet.payload, packet.qos, packet.retain);
                    // printf("topic:%s ,payload:%s ,payloadlen:%d.\r\n",packet.topic.c_str(), (char*)packet.payload,packet.payloadlen);
                    printf("time: %ld  ,topic:%s ,payloadlen:%d.\r\n", getCurrenTimeMs(), packet.topic.c_str(), packet.payloadlen);
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
            }
        }
    }

    void MqttClient::messageDecodeLoop()
    {
        while (loop_flag_)
        {
            if (recv_messges_vec_.size() > 0)
            {
                decode_msg_mtx_.lock();
                std::string packet = recv_messges_vec_.front();
                recv_messges_vec_.pop_back();
                decode_msg_mtx_.unlock();

                if (msg_callback_)
                {
                    msg_callback_(packet);
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    void MqttClient::on_connect(int rc)
    {
        printf("Connected with code %d.\n", rc);
        if (rc == 0)
        {
            /* Only attempt to subscribe on a successful connect. */
            //subscribe(NULL, "temperature/celsius");
            connect_flag_ = true;
            for (auto iter = sub_topic_vec_.cbegin(); iter != sub_topic_vec_.cend(); iter++)
            {
                subscribe(NULL, (*iter).c_str());
            }
        }
    }

    void MqttClient::on_message(const struct mosquitto_message *message)
    {
        // printf("recive message topic with: %s\n", message->topic);
        std::string topic = message->topic;
        if (std::find(sub_topic_vec_.begin(), sub_topic_vec_.end(), topic) != sub_topic_vec_.end())
        {
            // 线程处理
            // std::string parse_str = (char *)message->topic;
            // decode_msg_mtx_.lock();
            // recv_messges_vec_.push_back(parse_str);
            // decode_msg_mtx_.unlock();

            if(msg_callback_){
                std::string topic = message->topic;
                msg_callback_(topic);
            }

            if(pcd_callback_){
                std::string parse_str = (char *)message->payload;
                protocol::Protocol prtc;
                std::vector<PointType> pcd;
                prtc.decodeProtocolJson(parse_str, pcd);
                pcd_callback_(pcd);
            }

            if(imu_callback_){
                std::string parse_str = (char *)message->payload;
                protocol::Protocol prtc;
                easyrosbag::ImuOutput imu;
                prtc.decodeProtocolJson(parse_str, imu);
                imu_callback_(imu);
            }

            if(pose_callback_){
                std::string parse_str = (char *)message->payload;
                protocol::Protocol prtc;
                easyrosbag::Pose pose;
                prtc.decodeProtocolJson(parse_str, pose);
                pose_callback_(pose);
            }
        }
    }
} // namespace communication

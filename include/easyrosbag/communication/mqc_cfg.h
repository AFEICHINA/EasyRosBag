
#ifndef __MQC_CFG_H__
#define __MQC_CFG_H__
#include <string>
#include <stdint.h>
namespace communication
{
    struct MqcCfg
    {
        std::string ip = "192.168.1.26";
        int16_t port = 1883;
        int16_t keepalive = 60;
        MqcCfg(){};
        MqcCfg(std::string m_ip, int16_t m_port, int16_t m_keepalive)
        {
            ip = m_ip;
            port = m_port;
            keepalive = m_keepalive;
        }
    };

    struct PubMessge
    {
        std::string topic = "test";
        void *payload = NULL;
        int payloadlen = 0;
        bool retain = false;
        int qos = 0;

        PubMessge(std::string m_topic, void *m_payload, int m_payloadlen, bool m_retain, int m_qos)
        {
            topic = m_topic;
            payload = m_payload;
            payloadlen = m_payloadlen;
            retain = m_retain;
            qos = m_qos;
        }
    };
} // namespace communication
#endif
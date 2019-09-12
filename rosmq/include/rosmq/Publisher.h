#pragma once

#include "Common.h"

#include <ros/ros.h>
#include <ros/serialization.h>
#include <zmq.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace rosmq
{

/**
 * Each Publisher can only send via one protocol (see Common.h -> capnzero::Protocol), but it is possible to send
 * to multiple addresses via this protocol at once. Furthermore each message is send on a certain topic. For IPC and TCP
 * the topic is set via multi-part messages (see http://wiki.zeromq.org/blog:zero-copy), in case of UDP via multicast
 * groups (see http://api.zeromq.org/master:zmq-socket#toc6 ZMQ_RADIO/ZMQ_DISH socket type).
 */
class Publisher
{
public:
    Publisher(void* context, Protocol protocol);
    virtual ~Publisher();

    void setDefaultTopic(std::string defaultTopic);

    /**
     * Connects or binds the socket of the publisher to the given address.
     * @param address The address.
     */
    void addAddress(std::string address);

    /**
     * Sends the message to the given topic.
     * @param msgBuilder Builder containing the message.
     * @param topic The topic to send the message to.
     * @return Number of bytes sent.
     */
    template <typename M>
    int send(const M& m, std::string topic)
    {
        assert(topic.length() < MAX_TOPIC_LENGTH && "Publisher::send: The given topic is too long!");

        // setup zmq msg
        zmq_msg_t msg;
        int sumBytesSend = 0;

        uint32_t serial_size = ros::serialization::serializationLength(*m);
        zmq_msg_init_size(&msg, serial_size);
        ros::serialization::OStream stream((uint8_t*)zmq_msg_data(&msg), serial_size);
        ros::serialization::serialize(stream, *m);

        // Topic handling
        if (this->protocol == Protocol::UDP) {
            // for UDP via multicast group
            check(zmq_msg_set_group(&msg, topic.c_str()), "zmq_msg_set_group");
        } else {
            // for NON-UDP via multi part messages
            zmq_msg_t topicMsg;
            check(zmq_msg_init_data(&topicMsg, &topic, topic.size() * sizeof(topic), NULL, NULL), "zmq_msg_init_data for topic");
            sumBytesSend = checkSend(zmq_msg_send(&topicMsg, this->socket, ZMQ_SNDMORE), topicMsg, "Publisher-topic");
            if (sumBytesSend == 0) {
                // sending topic did not work, so stop here
                return sumBytesSend;
            }
        }

        sumBytesSend += checkSend(zmq_msg_send(&msg, this->socket, 0), msg, "Publisher-content");
        return sumBytesSend;
    }

    /**
     * Sends the message to the default topic.
     * @param msgBuilder Builder containing the message.
     * @return Number of bytes sent.
     */
    template <typename M>
    int send(const M& m)
    {
        return this->send(m, this->defaultTopic);
    }

protected:
    void* context;
    void* socket;
    std::string defaultTopic;
    Protocol protocol;
};

} // namespace rosmq

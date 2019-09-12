#pragma once

#include "Common.h"

#include <ros/ros.h>
#include <ros/serialization.h>
#include <zmq.h>

#include <string>

#include <assert.h>
#include <functional>
#include <thread>
#include <unistd.h>
#include <vector>

//#define DEBUG_SUBSCRIBER

namespace rosmq
{
template <typename MessageType>
class Subscriber
{
public:
    //    typedef std::function<void(::capnp::FlatArrayMessageReader&)> callbackFunction;
    using callbackFunction = void (*)(MessageType& m);

    Subscriber(void* context, Protocol protocol);
    virtual ~Subscriber();

    /**
     * Starts the receiving thread, if called for the first time. Changes the callback to the given function and object.
     * @tparam CallbackObjType
     * @param callbackFunction
     * @param callbackObject
     */
    //    template <class CallbackObjType, typename MessageType>
    template <class CallbackObjType>
    void subscribe(void (CallbackObjType::*callbackFunction)(MessageType& m), CallbackObjType* callbackObject)
    {
        using std::placeholders::_1;
        this->callbackFunction_ = std::bind(callbackFunction, callbackObject, _1);
        if (!running) {
            this->running = true;
            this->runThread = new std::thread(&Subscriber::receive, this);
        }
    }

    /**
     * Starts the receiving thread, if called for the first time. Changes the callback to the given function.
     * @param callbackFunction
     */
    void subscribe(void (*callbackFunction)(MessageType& m))
    {
        this->callbackFunction_ = callbackFunction;
        if (!running) {
            this->running = true;
            this->runThread = new std::thread(&Subscriber::receive, this);
        }
    }

    /**
     * Sets the topic to receive from.
     * @param defaultTopic
     */
    void setTopic(std::string topic);
    void addAddress(std::string address);

protected:
    void* context;
    void* socket;
    std::string topic;
    Protocol protocol;
    int rcvTimeout; /** < only initialized if needed */

    callbackFunction callbackFunction_;

    std::thread* runThread;
    bool running;

    void receive();
};

template <typename MessageType>
Subscriber<MessageType>::Subscriber(void* context, Protocol protocol)
        : socket(nullptr)
        , rcvTimeout(500)
        , topic("???") // this filter topic will hopefully never be used
        , context(context)
        , protocol(protocol)
        , running(false)
        , runThread(nullptr)
        , callbackFunction_(nullptr)
{
    // set socket type with respect to given protocol
    switch (this->protocol) {
    case Protocol::UDP:
        this->socket = zmq_socket(this->context, ZMQ_DISH);
        break;
    case Protocol::TCP:
    case Protocol::IPC:
        this->socket = zmq_socket(this->context, ZMQ_SUB);
        break;
    default:
        // Unknown protocol!
        assert(false && "Subscriber::Subscriber: The given protocol is unknown!");
    }

    check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
}

template <typename MessageType>
Subscriber<MessageType>::~Subscriber()
{
    this->running = false;
    this->runThread->join();
    delete this->runThread;
    check(zmq_close(this->socket), "zmq_close");
}

template <typename MessageType>
void Subscriber<MessageType>::setTopic(std::string topic)
{
    assert(topic.length() < MAX_TOPIC_LENGTH && "Subscriber::setTopic: The given topic is too long!");

    if (this->topic.compare(topic) == 0) {
        return;
    }

    switch (this->protocol) {
    case Protocol::UDP:
        this->topic = topic;
        check(zmq_join(this->socket, this->topic.c_str()), "zmq_join");
        break;
    case Protocol::TCP:
    case Protocol::IPC:
        if (this->topic.compare("???") != 0) {
            check(zmq_setsockopt(this->socket, ZMQ_UNSUBSCRIBE, this->topic.c_str(), 0), "zmq_setsockopt");
        }
        this->topic = topic;
        check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, this->topic.c_str(), 0), "zmq_setsockopt");
        break;
    default:
        // Unknown protocol!
        assert(false && "Subscriber::setTopic: The given protocol is unknown!");
    }
}
template <typename MessageType>
void Subscriber<MessageType>::addAddress(std::string address)
{
    switch (protocol) {
    case Protocol::UDP:
        check(zmq_bind(this->socket, ("udp://" + address).c_str()), "zmq_bind");
        break;
    case Protocol::TCP:
        check(zmq_bind(this->socket, ("tcp://" + address).c_str()), "zmq_bind");
        break;
    case Protocol::IPC:
        check(zmq_bind(this->socket, ("ipc://" + address).c_str()), "zmq_bind");
        break;
    default:
        // Unknown protocol!
        assert(false && "Subscriber::addAddress: Given protocol is unknown!");
    }
}

template <typename MessageType>
void Subscriber<MessageType>::receive()
{
    while (this->running) {

#ifdef DEBUG_SUBSCRIBER
        std::cout << "Subscriber::received() waiting ..." << std::endl;
#endif

        if (this->protocol != Protocol::UDP) {
            // Topic handling for non-udp protocols via multipart message
            zmq_msg_t topicMsg;
            check(zmq_msg_init(&topicMsg), "zmq_msg_init");
            if (0 == checkReceive(zmq_msg_recv(&topicMsg, this->socket, ZMQ_SNDMORE), topicMsg, "Subscriber::receive-Topic")) {
                // error or timeout on recv
                //                std::cout << "Subscriber: No Message here!" << std::endl;
                continue;
            }
        }

        zmq_msg_t msg;
        check(zmq_msg_init(&msg), "zmq_msg_init");
        if (0 == checkReceive(zmq_msg_recv(&msg, this->socket, 0), msg, "Subscriber::receive")) {
            // error or timeout on recv
            //            std::cout << "Subscriber: No Message here!" << std::endl;
            continue;
        }

#ifdef DEBUG_SUBSCRIBER
        std::cout << std::endl;
#endif

        // Received message must contain an integral number of words.
        //        if (zmq_msg_size(&msg) % Subscriber::WORD_SIZE != 0) {
        //            std::cerr << "Subscriber::receive(): Message received with a size of non-integral number of words!" << std::endl;
        //            check(zmq_msg_close(&msg), "zmq_msg_close");
        //            continue;
        //        }

        // Check whether message is memory aligned
        //        assert(reinterpret_cast<uintptr_t>(zmq_msg_data(&msg)) % Subscriber::WORD_SIZE == 0);

        // Call the callback with Cap'n Proto message
        ros::serialization::IStream stream((uint8_t*) zmq_msg_data(&msg), zmq_msg_size(&msg));
        MessageType rosMsg;
        ros::serialization::Serializer<MessageType>::read(stream, rosMsg);
        (this->callbackFunction_)(rosMsg);

        check(zmq_msg_close(&msg), "zmq_msg_close");
    }
}

} // namespace rosmq

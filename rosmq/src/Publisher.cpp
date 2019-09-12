#include "rosmq/Publisher.h"

#include <assert.h>

namespace rosmq
{

/**
 * Static method that cleans up sent messages. The method is called by ZeroMQ!
 * @param data Pointer to the data that was sent.
 * @param hint
 */
static void cleanUpMsgData(void* data, void* hint);

Publisher::Publisher(void* context, Protocol protocol)
        : socket(nullptr)
        , protocol(protocol)
        , context(context)
        , defaultTopic("")
{
    // set socket type with respect to given protocol
    switch (this->protocol) {
    case Protocol::UDP:
        this->socket = zmq_socket(this->context, ZMQ_RADIO);
        break;
    case Protocol::TCP:
    case Protocol::IPC:
        this->socket = zmq_socket(this->context, ZMQ_PUB);
        break;
    default:
        // Unknown protocol!
        assert(false && "Publisher::Publisher: The given protocol is unknown!");
    }
}

Publisher::~Publisher()
{
    check(zmq_close(this->socket), "zmq_close");
}

void Publisher::setDefaultTopic(std::string defaultTopic)
{
    assert(defaultTopic.length() < MAX_TOPIC_LENGTH && "Publisher::setTopic: The given default topic is too long!");
    this->defaultTopic = defaultTopic;
}

void Publisher::addAddress(std::string address)
{
    switch (this->protocol) {
    case Protocol::UDP:
        check(zmq_connect(this->socket, ("udp://" + address).c_str()), "zmq_connect");
        break;
    case Protocol::TCP:
        check(zmq_connect(this->socket, ("tcp://" + address).c_str()), "zmq_connect");
        break;
    case Protocol::IPC:
        check(zmq_connect(this->socket, ("ipc://" + address).c_str()), "zmq_connect");
        break;
    default:
        // Unknown protocol!
        assert(false && "Publisher::Publisher: The given protocol is unknown!");
    }
}

//static void cleanUpMsgData(void* data, void* hint)
//{
//    delete hint;
//}
} // namespace rosmq

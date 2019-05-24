#include "capnzero/Publisher.h"

#include <assert.h>

namespace capnzero
{

Publisher::Publisher(void* context)
        : socket(nullptr)
        , commType(commType)
        , context(context)
{
    this->groupName = "NONE";
}

Publisher::~Publisher()
{
    check(zmq_close(this->socket), "zmq_close");
}

void Publisher::setDefaultGroup(std::string group)
{
    this->groupName = group;
}

std::string Publisher::getDefaultGroup()
{
    return this->groupName;
}

void Publisher::bind(CommType commType, std::string address)
{
    switch (commType) {
    case CommType::UDP:
        this->commType = commType;
        this->socket = zmq_socket(this->context, ZMQ_RADIO);
        check(zmq_connect(this->socket, ("udp://" + address).c_str()), "zmq_connect");
        break;
    case CommType::TCP:
        this->socket = zmq_socket(this->context, ZMQ_PUB);
        check(zmq_bind(this->socket, ("tcp://" + address).c_str()), "zmq_bind");
        break;
    case CommType::IPC:
        this->socket = zmq_socket(this->context, ZMQ_PUB);
        check(zmq_bind(this->socket, ("ipc://" + address).c_str()), "zmq_bind");
        break;
    default:
        // Unknown communication type!
        assert(false);
    }
}

static void cleanUpMsgData(void* data, void* hint)
{
    delete reinterpret_cast<kj::Array<capnp::word>*>(hint);
}

int Publisher::send(::capnp::MallocMessageBuilder& msgBuilder, std::string topic)
{
    // setup zmq msg
    zmq_msg_t msg;

    kj::Array<capnp::word> wordArray = capnp::messageToFlatArray(msgBuilder);
    kj::Array<capnp::word>* wordArrayPtr = new kj::Array<capnp::word>(kj::mv(wordArray)); // will be delete by zero-mq
    check(zmq_msg_init_data(&msg, wordArrayPtr->begin(), wordArrayPtr->size() * sizeof(capnp::word), &cleanUpMsgData, wordArrayPtr), "zmq_msg_init_data");

    // set group
    if (this->commType == capnzero::CommType::UDP) {
//        std::cout << "Publisher: Sending on Group '" << topic << "'" << std::endl;
        check(zmq_msg_set_group(&msg, topic.c_str()), "zmq_msg_set_group");
    }

    // send
    if (this->commType != capnzero::CommType::UDP) {
        zmq_msg_t topicMsg;
        zmq_msg_init_data(&topicMsg, &topic, topic.size() * sizeof(topic), &cleanUpMsgData, NULL);
        zmq_msg_send(&topicMsg, this->socket, ZMQ_SNDMORE);
    }
    int numBytesSend = zmq_msg_send(&msg, this->socket, 0);
    if (numBytesSend == -1) {
        std::cerr << "zmq_msg_send was unsuccessful: Errno " << errno << " means: " << zmq_strerror(errno) << std::endl;
        check(zmq_msg_close(&msg), "zmq_msg_close");
    }

    return numBytesSend;
}

int Publisher::send(::capnp::MallocMessageBuilder& msgBuilder)
{
    return this->send(msgBuilder, this->getDefaultGroup());
}
} // namespace capnzero

#pragma once

#include "capnzero/Common.h"

#include <capnp/serialize-packed.h>

#include <zmq.h>

#include <string>

#include <assert.h>
#include <functional>
#include <thread>
#include <unistd.h>

//#define DEBUG_SUBSCRIBER

namespace capnzero
{

class Subscriber
{
public:
    typedef std::function<void(::capnp::FlatArrayMessageReader&)> callbackFunction;

    Subscriber(void* context, std::string groupName)
            : socket(nullptr)
            , groupName(groupName)
            , callbackFunction_(nullptr)
            , running(true)
            , runThread(nullptr)
            , rcvTimeout(500)
    {
        //        this->socket = zmq_socket(context, ZMQ_SUB);
        this->socket = zmq_socket(context, ZMQ_DISH);
        check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
//        check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, "", 0), "zmq_setsockopt");
        //        check(zmq_join(this->socket, this->groupName.c_str()), "zmq_join");
    }

    virtual ~Subscriber();

    void connect(CommType commType, std::string address);

    template <class CallbackObjType>
    void subscribe(void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader&), CallbackObjType* callbackObject);

    void subscribe(void (*callbackFunction)(::capnp::FlatArrayMessageReader&));

    static const int wordSize;

    callbackFunction callbackFunction_;

protected:
    void* socket;
    int rcvTimeout;
    std::string groupName;
    std::thread* runThread;
    bool running;

    void receive();
};

const int Subscriber::wordSize = sizeof(capnp::word);

Subscriber::~Subscriber()
{
    this->running = false;
    this->runThread->join();
    delete this->runThread;
    check(zmq_close(this->socket), "zmq_close");
}

void Subscriber::connect(CommType commType, std::string address)
{
    switch (commType) {
    case CommType::UDP:
        check(zmq_connect(this->socket, ("udp://" + address).c_str()), "zmq_bind");
        break;
    case CommType::TCP:
        check(zmq_connect(this->socket, ("tcp://" + address).c_str()), "zmq_bind");
        break;
    case CommType::IPC:
        check(zmq_connect(this->socket, ("ipc://" + address).c_str()), "zmq_bind");
        break;
    default:
        // Unknown communication type!
        assert(false);
    }
}

template <class CallbackObjType>
void Subscriber::subscribe(void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader&), CallbackObjType* callbackObject)
{
    using std::placeholders::_1;
    this->callbackFunction_ = std::bind(callbackFunction, callbackObject, _1);
    this->runThread = new std::thread(&Subscriber::receive, this);
}

void Subscriber::subscribe(void (*callbackFunction)(::capnp::FlatArrayMessageReader&))
{
    this->callbackFunction_ = callbackFunction;
    this->runThread = new std::thread(&Subscriber::receive, this);
}

void Subscriber::receive()
{
    while (this->running) {
        zmq_msg_t msg;
        check(zmq_msg_init(&msg), "zmq_msg_init");
#ifdef DEBUG_SUBSCRIBER
        std::cout << "Subscriber::received() waiting ..." << std::endl;
#endif

        int nbytes = zmq_msg_recv(&msg, this->socket, 0);

        //        std::cout << "Subscriber::receive(): nBytes: " << nbytes << " errno: " << errno << "(EAGAIN: " << EAGAIN << ")" << std::endl;

        // handling for unsuccessful call to zmq_msg_recv
        if (nbytes == -1) {
            if (errno != EAGAIN) // receiving a message was unsuccessful
            {
                std::cerr << "Subscriber::receive(): zmq_msg_recv returned: -1 - " << zmq_strerror(errno) << std::endl;
            }
#ifdef DEBUG_SUBSCRIBER
            else // no message available
            {
                //                std::cout << "Subscriber::receive(): continue because of EAGAIN!" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
#endif

            std::cout << ".";
            std::cout.flush();
            check(zmq_msg_close(&msg), "zmq_msg_close");
            continue;
        } else {
            std::cout << std::endl;
        }

        // Received message must contain an integral number of words.
        if (zmq_msg_size(&msg) % Subscriber::wordSize != 0) {
            std::cout << "Non-Integral number of words!" << std::endl;
            check(zmq_msg_close(&msg), "zmq_msg_close");
            continue;
        }

        // Check whether message is memory aligned
        assert(reinterpret_cast<uintptr_t>(zmq_msg_data(&msg)) % Subscriber::wordSize == 0);

        int numWordsInMsg = zmq_msg_size(&msg);
        auto wordArray = kj::ArrayPtr<capnp::word const>(reinterpret_cast<capnp::word const*>(zmq_msg_data(&msg)), numWordsInMsg);

        ::capnp::FlatArrayMessageReader msgReader = ::capnp::FlatArrayMessageReader(wordArray);

        (this->callbackFunction_)(msgReader);

        check(zmq_msg_close(&msg), "zmq_msg_close");
    }
}
} // namespace capnzero

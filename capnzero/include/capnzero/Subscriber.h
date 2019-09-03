#pragma once

#include "capnzero/Common.h"

#include <capnp/serialize-packed.h>

#include <zmq.h>

#include <string>

#include <assert.h>
#include <functional>
#include <thread>
#include <unistd.h>
#include <vector>

//#define DEBUG_SUBSCRIBER

namespace capnzero {

    struct Address {
        std::string address;
        Protocol type;

        Address(std::string addr, Protocol t) : address(addr), type(t) {}
    };

    class Subscriber {
    public:
        typedef std::function<void(::capnp::FlatArrayMessageReader & )> callbackFunction;

        Subscriber(void *context, std::string groupName, void (*callbackFunction)(::capnp::FlatArrayMessageReader &))
                : socket(nullptr), udpSocket(nullptr), topic(groupName), callbackFunction_(callbackFunction),
                  running(true), runThread(nullptr), rcvTimeout(500), context(context) {
            std::cout << "Group: " << this->topic << std::endl;
            this->isConnected = false;
        }

        template<class CallbackObjType>
        Subscriber(void *context, std::string groupName,
                   void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader &),
                   CallbackObjType *callbackObject)
                : socket(nullptr), udpSocket(nullptr), topic(groupName), running(true), runThread(nullptr),
                  rcvTimeout(500), context(context) {
            using std::placeholders::_1;
            this->callbackFunction_ = std::bind(callbackFunction, callbackObject, _1);
            std::cout << "Group: " << this->topic << std::endl;
            this->isConnected = false;
        }

        virtual ~Subscriber();

        void addAddress(Protocol protocol, std::string addr);

        void connect();

        static const int wordSize;

        callbackFunction callbackFunction_;

    protected:
        void *context;
        void *socket;
        void *udpSocket;        // This was made in the assumption a socket cannot be of type SUB and DISH at the same time.
        // only initialized if needed
        int rcvTimeout;
        std::string topic;
        std::thread *runThread;
        bool running;
        bool hasUDP;
        bool isConnected;
        std::vector <Address> addresses;

        void receive();
    };

} // namespace capnzero

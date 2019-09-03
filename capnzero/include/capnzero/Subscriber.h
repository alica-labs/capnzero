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

namespace capnzero
{

struct Address
{
    std::string address;
    Protocol protocol;

    Address(std::string address, Protocol protocol)
            : address(address)
            , protocol(protocol)
    {
    }
};

class Subscriber
{
public:
    typedef std::function<void(::capnp::FlatArrayMessageReader&)> callbackFunction;

    Subscriber(void* context, std::string defaultTopic, void (*callbackFunction)(::capnp::FlatArrayMessageReader&))
            : socket(nullptr)
            , defaultTopic(defaultTopic)
            , callbackFunction_(callbackFunction)
            , running(true)
            , runThread(nullptr)
            , rcvTimeout(500)
            , context(context)
    {
        std::cout << "Topic: " << this->defaultTopic << std::endl;
    }

    template <class CallbackObjType>
    Subscriber(void* context, std::string defaultTopic, void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader&),
            CallbackObjType* callbackObject)
            : socket(nullptr)
            , defaultTopic(defaultTopic)
            , running(true)
            , runThread(nullptr)
            , rcvTimeout(500)
            , context(context)
    {
        using std::placeholders::_1;
        this->callbackFunction_ = std::bind(callbackFunction, callbackObject, _1);
        std::cout << "Topic: " << this->defaultTopic << std::endl;
    }

    virtual ~Subscriber();

    void addAddress(std::string address);
    void connect();
    static const int wordSize;
    callbackFunction callbackFunction_;

protected:
    void* context;
    void* socket;
    std::string defaultTopic;
    Protocol protocol;
    int rcvTimeout; /** < only initialized if needed */

    std::vector<Address> addresses;

    std::thread* runThread;
    bool running;

    void receive();
};

} // namespace capnzero

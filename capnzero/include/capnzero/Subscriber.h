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

struct Address{
    std::string address;
    CommType type;
};

class Subscriber
{
public:
    typedef std::function<void(::capnp::FlatArrayMessageReader&)> callbackFunction;

    Subscriber(void *context, std::string groupName, std::string defaultAddress, void (*callbackFunction)(::capnp::FlatArrayMessageReader&))
            : socket(nullptr)
            , groupName(groupName)
            , callbackFunction_(callbackFunction)
            , running(true)
            , runThread(nullptr)
            , rcvTimeout(500)
            , context(context)
    {
        std::cout << "Group: " << this->groupName << std::endl;
    }

    template <class CallbackObjType>
    Subscriber(void* context, std::string groupName, void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader&), CallbackObjType* callbackObject)
            : socket(nullptr)
            , groupName(groupName)
            , running(true)
            , runThread(nullptr)
            , rcvTimeout(500)
            , context(context)
    {
        using std::placeholders::_1;
        this->callbackFunction_ = std::bind(callbackFunction, callbackObject, _1);
        std::cout << "Group: " << this->groupName << std::endl;
    }

    virtual ~Subscriber();

    void add_address(CommType commType, std::string address);

    void connect(CommType commType);

    static const int wordSize;

    callbackFunction callbackFunction_;

protected:
    void* context;
    void* socket;
    int rcvTimeout;
    std::string groupName;
    std::thread* runThread;
    bool running;
    bool hasUDP;
    std::vector<address> addresses;

    void receive();
};

} // namespace capnzero

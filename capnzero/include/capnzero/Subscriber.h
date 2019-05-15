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
            , context(context)
    {
    }

    virtual ~Subscriber();

    void connect(CommType commType, std::string address);

    template <class CallbackObjType>
    void subscribe(void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader&), CallbackObjType* callbackObject);

    void subscribe(void (*callbackFunction)(::capnp::FlatArrayMessageReader&));

    static const int wordSize;

    callbackFunction callbackFunction_;

protected:
    void* context;
    void* socket;
    int rcvTimeout;
    std::string groupName;
    std::thread* runThread;
    bool running;
    capnzero::CommType type;

    void receive();
};

template <class CallbackObjType>
void Subscriber::subscribe(void (CallbackObjType::*callbackFunction)(::capnp::FlatArrayMessageReader&), CallbackObjType* callbackObject)
{
    using std::placeholders::_1;
    this->callbackFunction_ = std::bind(callbackFunction, callbackObject, _1);
    this->runThread = new std::thread(&Subscriber::receive, this);
}
} // namespace capnzero

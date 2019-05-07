#pragma once

#include "Common.h"
#include <capnp/serialize-packed.h>
#include <zmq.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace capnzero
{

class Publisher
{
public:
    Publisher(void* context);
    virtual ~Publisher();

    void setDefaultGroup(std::string group);
    std::string getDefaultGroup();

    void bind(CommType commType, std::string address);

    int send(capnp::MallocMessageBuilder& msgBuilder);
    int send(capnp::MallocMessageBuilder& msgBuilder, std::string topic);

protected:
    void* context;
    void* socket;
    std::string groupName;
    CommType commType;
};

//static void cleanUpMsgData(void* data, void* hint);
} // namespace capnzero

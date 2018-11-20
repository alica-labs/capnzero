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
    Publisher(void* context, std::string groupName);
    virtual ~Publisher();

    void bind(CommType commType, std::string address);

    int send(capnp::MallocMessageBuilder& msgBuilder);

protected:
    void* socket;
    std::string groupName;
    CommType commType;
};

static void cleanUpMsgData(void* data, void* hint);
} // namespace capnzero

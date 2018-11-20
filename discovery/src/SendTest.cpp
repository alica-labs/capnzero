#include <capnzero/Publisher.h>
#include <capnzero/Subscriber.h>

#include <discovery_msgs/beacon.capnp.h>
#include <zmq.h>

#include <uuid/uuid.h>

#include <assert.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>

#include <iostream>
#include <memory>
#include <unistd.h>

#include <signal.h>

bool stop;
void* ctx;
int msgNumber;
capnzero::Publisher* pub;
capnzero::Subscriber* sub;

/**
 * Checks the return code and reports an error if present.
 * If abortIfError is set to true, it also aborts the process.
 */
void check(int returnCode, std::string methodName, bool abortIfError)
{
    if (returnCode != 0) {
        std::cerr << methodName << " returned: " << errno << " - " << zmq_strerror(errno) << std::endl;
        if (abortIfError)
            assert(returnCode);
    }
}

void send(capnzero::Publisher* pub, int msgNumber)
{
    capnp::MallocMessageBuilder msgBuilder;
    discovery_msgs::Beacon::Builder beaconMsgBuilder = msgBuilder.initRoot<discovery_msgs::Beacon>();

    // set content
    beaconMsgBuilder.setIp("192.186.0.1");
    beaconMsgBuilder.setPort(msgNumber);
    uuid_t uuid;
    uuid_generate(uuid);
    beaconMsgBuilder.setUuid(kj::arrayPtr(uuid, sizeof(uuid)));

    std::cout << "SendTest::send(): Message sent: '" << beaconMsgBuilder.toString().flatten().cStr() << "'" << std::endl;
    int numBytesSent = pub->send(msgBuilder);

    // std::cout << "SendTest::send(): " << numBytesSent << " Bytes sent! " << std::endl;
}

void receive(::capnp::FlatArrayMessageReader& reader)
{
    std::cout << "SendTest::receive(..) called!" << std::endl;
    auto beacon = reader.getRoot<discovery_msgs::Beacon>();

    if (beacon.getPort() != msgNumber) {
        std::cout << "SendTest::receive(..): Message received: '" << beacon.toString().flatten().cStr() << "'" << std::endl;

        msgNumber = beacon.getPort();
        if (msgNumber != 100) {
            send(pub, ++msgNumber);
        }
    } else {
        std::cout << "SendTest::receive(..): Received own Number " << std::endl;
    }
}

void sigIntHandler(int sig)
{
    stop = true;
}

int main(int argc, char** argv)
{
    stop = false;
    // register ctrl+c handler
    signal(SIGINT, sigIntHandler);

    ctx = zmq_ctx_new();
    assert(ctx);

    pub = new capnzero::Publisher(ctx, "MCGroup");
    pub->bind(capnzero::CommType::UDP, "udp://224.0.0.1:5555");
    sub = new capnzero::Subscriber(ctx, "MCGroup");
    sub->connect(capnzero::CommType::UDP, "udp://224.0.0.1:5555");
    sub->subscribe(&receive);

    msgNumber = 0;
    if (argc > 1 && strcmp(argv[1], "sender") == 0) {
        send(pub, ++msgNumber);
    }

    while (!stop && msgNumber != 100) {
        usleep(1);
    }

    delete pub;
    delete sub;
    check(zmq_ctx_term(ctx), "zmq_ctx_term", true);
}

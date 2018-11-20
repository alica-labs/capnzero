#include "capnzero-base-msgs/string.capnp.h"

#include <capnzero/Publisher.h>
#include <capnzero/Common.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <chrono>
#include <thread>
#include <signal.h>

#define DEBUG_PUB

static bool interrupted = false;
static void s_signal_handler(int signal_value)
{
    interrupted = true;
}

static void s_catch_signals(void)
{
    struct sigaction action;
    action.sa_handler = s_signal_handler;
    action.sa_flags = 0;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
}

int main(int argc, char** argv)
{
    s_catch_signals();

    if (argc <= 1) {
        std::cerr << "Synopsis: rosrun capnzero pub \"String that should be published!\"" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    // Cap'n Proto: create proto message

    // init builder
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero::String::Builder beaconMsgBuilder = msgBuilder.initRoot<capnzero::String>();

    // set content
    beaconMsgBuilder.setString(argv[2]);

#ifdef DEBUG_PUB
    std::cout << "pub: Message to send: " << beaconMsgBuilder.toString().flatten().cStr() << std::endl;
#endif

    void* ctx = zmq_ctx_new();
    capnzero::Publisher pub = capnzero::Publisher(ctx, argv[1]);
    pub.bind(capnzero::CommType::IPC, "@capnzero.ipc");
    while (!interrupted) {
        int numBytesSent = pub.send(msgBuilder);
#ifdef DEBUG_PUB
        std::cout << "pub: " << numBytesSent << " Bytes sent!" << std::endl;
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }



    // wait until everything is send
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

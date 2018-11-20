#include "capnzero-base-msgs/string.capnp.h"

#include <capnzero/Common.h>
#include <capnzero/Subscriber.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>

#include <signal.h>
#include <thread>

#define DEBUG_SENDER

void callback(::capnp::FlatArrayMessageReader& reader)
{
    std::cout << "Called callback..." << std::endl;
    std::cout << reader.getRoot<capnzero::String>().toString().flatten().cStr() << std::endl;
}

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
        std::cerr << "Synopsis: rosrun capnzero echo \"Topic that should be listened to!\"" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    void* ctx = zmq_ctx_new();
    capnzero::Subscriber sub = capnzero::Subscriber(ctx, argv[1]);
    sub.connect(capnzero::CommType::IPC, "@capnzero.ipc");
    sub.subscribe(&callback);

    while (!interrupted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}

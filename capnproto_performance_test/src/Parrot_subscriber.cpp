#include "capnproto-base-msgs/string.capnp.h"
#include <capnzero/Common.h>
#include <capnzero/Subscriber.h>
#include <capnzero/Publisher.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <signal.h>
#include <thread>
#include <vector>
#include <string>
#include <bitset>
#include <nl_types.h>
std::string  rcvmsgstring;
int16_t rcvmsgnumber;
#define DEBUG_SENDER
void callback(::capnp::FlatArrayMessageReader& reader);
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
    capnproto::Subscriber* sub = new capnproto::Subscriber(ctx, argv[1]);
    capnproto::Publisher pub = capnproto::Publisher(ctx, argv[1]);
    sub->connect(capnproto::CommType::UDP, "224.0.0.2:5500");
    sub->subscribe(&callback);
    // init builder
    ::capnp::MallocMessageBuilder msgBuilder;
    capnproto::Capnprotoperformancetest::Builder dataHolder= msgBuilder.initRoot<capnproto::Capnprotoperformancetest>();
    //Publisher  part
    pub.bind(capnproto::CommType::UDP, "224.0.0.2:5554");
    while (!interrupted) {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        int numBytesSent = pub.send(msgBuilder);
        {
            std::cout << "I am going to publish the following message: "<< numBytesSent << " Bytes sent!" << std::endl;
            dataHolder.setMessage(rcvmsgstring);
            dataHolder.setNumber(rcvmsgnumber);
            rcvmsgstring.clear();
            dataHolder.hasMessage();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    delete sub;
    zmq_ctx_term(ctx);
}

void callback(::capnp::FlatArrayMessageReader& reader) {
    std::cout << "Subscriber called for port 5500 and rcvd message: " << std::endl;
    reader.getRoot<capnproto::Capnprotoperformancetest>().toString().flatten().cStr();
    rcvmsgstring=reader.getRoot<capnproto::Capnprotoperformancetest>().getMessage();
    rcvmsgnumber=int16_t (reader.getRoot<capnproto::Capnprotoperformancetest>().getNumber());
    std::cout << "Received string message: "<<rcvmsgstring << std::endl;
    std::cout << "Received int message: "<<rcvmsgnumber << std::endl;
}
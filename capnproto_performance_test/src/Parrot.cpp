#include "capnproto-base-msgs/Testmessage.capnp.h"
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

static std::string  rcvmsgstring; //Global variable
static long rcvmsgnumber=NULL; //Global variable
#define DEBUG_SENDER
void callback(::capnp::FlatArrayMessageReader& reader);

void sender (capnzero::Publisher *pub);
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

int main(int argc, char** argv) // Stack frame started
{
    s_catch_signals();

    if (argc <= 1) {
        std::cerr << "Synopsis: rosrun capnproto echo \"Topic that should be listened to!\"" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }
    void* ctx = zmq_ctx_new();
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, argv[1]); // creating a pointer in the heap
    capnzero::Publisher *pub = new capnzero::Publisher(ctx);
    pub->setDefaultGroup(argv[1]);
    sub->connect(capnzero::CommType::UDP, "224.0.0.2:5500");
    sub->subscribe(&callback);
    // init builder

    //Publisher  part

    while (!interrupted) {
        if(rcvmsgnumber != NULL)
        {
           sender(pub);

        }
        else{"subscriber didn't  get called";}
    }
    std::cout << "Cleaning up now. "  << std::endl;
    delete sub;
    delete pub;// dynamic memory deallocated
    zmq_ctx_term(ctx);
}

void callback(::capnp::FlatArrayMessageReader& reader)
{
    std::cout << "Subscriber called for port 5500 and rcvd message: " << std::endl;
    reader.getRoot<capnzero::Testmessage>().toString().flatten().cStr();
    rcvmsgstring= reader.getRoot<capnzero::Testmessage>().getPayload();
    rcvmsgnumber= int16_t (reader.getRoot<capnzero::Testmessage>().getId()); //Type casting
    std::cout << "Received string message: "<<rcvmsgstring << std::endl;
    std::cout << "Received int message: "<<rcvmsgnumber << std::endl;
}
void sender (capnzero::Publisher *pub){
    pub->bind(capnzero::CommType::UDP, "224.0.0.2:5554");
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero::Testmessage::Builder dataHolder= msgBuilder.initRoot<capnzero::Testmessage>();
    dataHolder.setPayload(rcvmsgstring);
    dataHolder.setId(rcvmsgnumber);
    int numBytesSent = pub->send(msgBuilder);
    {
        std::cout << "I am going to publish the following message: " << numBytesSent << " Bytes sent!"
                  << std::endl;

        dataHolder.hasPayload();
        rcvmsgnumber=NULL;
        rcvmsgstring.clear();

    }
};
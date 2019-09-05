#include "capnzero-eval-msgs/EvalMessage.capnp.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnzero/Common.h>
#include <capnzero/Publisher.h>
#include <capnzero/Subscriber.h>
#include <kj/array.h>
#include <signal.h>

#include <vector>

void callback(::capnp::FlatArrayMessageReader& reader);
capnzero::Publisher* pub;

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
        std::cerr << "Synopsis: rosrun capnproto echo \"Topic that should be listened to!\"" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    void* ctx = zmq_ctx_new();
//    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::UDP);
//    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::IPC);
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::TCP);
//    sub->addAddress("224.0.0.2:5500");
//    sub->addAddress("@capnzeroSend.ipc");
    sub->addAddress("127.0.0.1:5500");
    sub->setTopic(argv[1]);
    sub->subscribe(&callback);

//    pub = new capnzero::Publisher(ctx, capnzero::Protocol::UDP);
//    pub = new capnzero::Publisher(ctx, capnzero::Protocol::IPC);
    pub = new capnzero::Publisher(ctx, capnzero::Protocol::TCP);
//    pub->addAddress("@capnzeroReceive.ipc");
//    pub->addAddress("224.0.0.2:5554");
    pub->addAddress("127.0.0.1:5554");
    pub->setDefaultTopic(argv[1]);

    while (!interrupted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Cleaning up now. " << std::endl;
    delete sub;
    delete pub;
    zmq_ctx_term(ctx);
}

void callback(::capnp::FlatArrayMessageReader& reader)
{
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero_eval::EvalMessage::Builder msg = msgBuilder.initRoot<capnzero_eval::EvalMessage>();
    msg.setPayload(reader.getRoot<capnzero_eval::EvalMessage>().getPayload());
    msg.setId(reader.getRoot<capnzero_eval::EvalMessage>().getId());
    pub->send(msgBuilder);
}
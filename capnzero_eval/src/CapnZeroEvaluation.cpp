#include "ExperimentLog.h"
#include "capnzero-eval-msgs/EvalMessage.capnp.h"

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnzero/Common.h>
#include <capnzero/Publisher.h>
#include <capnzero/Subscriber.h>
#include <kj/array.h>

#include <chrono>
#include <signal.h>
#include <string>
#include <thread>

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

void callback(::capnp::FlatArrayMessageReader& reader);
ExperimentLog* experimentLog;

int main(int argc, char** argv)
{
    s_catch_signals();

    if (argc <= 1) {
        std::cerr << "Synopsis: rosrun capnproto pub \"String that should be published!\"" << std::endl;
        return -1;
    }
    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    void* ctx = zmq_ctx_new();
    //    capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::UDP);
    //    pub->addAddress("224.0.0.2:5500");
        capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::IPC);
        pub->addAddress("@capnzeroSend.ipc");
//    capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::TCP);
//    pub->addAddress("127.0.0.1:5500");

    pub->setDefaultTopic(argv[1]);

    //    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::UDP);
    //    sub->addAddress("224.0.0.2:5554");
        capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::IPC);
        sub->addAddress("@capnzeroReceive.ipc");
//    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::TCP);
//    sub->addAddress("127.0.0.1:5554");

    sub->setTopic(argv[1]);
    sub->subscribe(&callback);

    // Cap'n Zero: create proto message
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero_eval::EvalMessage::Builder beaconMsgBuilder = msgBuilder.initRoot<capnzero_eval::EvalMessage>();
    beaconMsgBuilder.setPayload(argv[2]);

    experimentLog = new ExperimentLog("results", "IPC");
    int16_t counter = 0;
    while (!interrupted && counter != 1000) {
        beaconMsgBuilder.setId(++counter);
        experimentLog->addStartedMeasurement(counter, std::chrono::high_resolution_clock::now());
        pub->send(msgBuilder);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // wait for messages to arrive, then write results to file
    std::this_thread::sleep_for(std::chrono::seconds(1));
    experimentLog->calcStatistics();
    experimentLog->serialise();

    std::cout << "Cleaning up now. " << std::endl;
    delete experimentLog;
    delete sub;
    delete pub;
    zmq_ctx_term(ctx);

    return 0;
}

void callback(::capnp::FlatArrayMessageReader& reader)
{
    experimentLog->finishMeasurement(reader.getRoot<capnzero_eval::EvalMessage>().getId(), std::chrono::high_resolution_clock::now());
}

#include "ExperimentLog.h"
#include "capnzero-eval-msgs/EvalMessageCapnZero.capnp.h"
#include "capnzero_eval/EvalMessageRos.h"

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnzero/Common.h>
#include <capnzero/Publisher.h>
#include <capnzero/Subscriber.h>
#include <kj/array.h>

#include <ros/ros.h>
#include <ros/serialization.h>

#include <chrono>
#include <random>
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

void evalCapnZero(std::string topic);
void callbackCapnZero(::capnp::FlatArrayMessageReader& reader);

void evalRos(int argc, char** argv, std::string topic);
void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg);

ExperimentLog* experimentLog;

int main(int argc, char** argv)
{
    s_catch_signals();

    if (argc <= 1) {
        std::cerr << "Synopsis: rosrun capnzero_eval Evaluate [ros|capnzero] \"topic\"" << std::endl;
        return -1;
    }
    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    if (std::string("ros").compare(argv[1]) == 0) {
        evalRos(argc, argv, argv[2]);
    } else {
        evalCapnZero(argv[2]);
    }

    std::cout << "Cleaning up now. " << std::endl;
    delete experimentLog;

    return 0;
}

void evalRos(int argc, char** argv, std::string topic)
{
    experimentLog = new ExperimentLog("results", "ROS");

    ros::init(argc, argv, "evalRos");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<capnzero_eval::EvalMessageRos>(topic, 1000);
    ros::Subscriber sub = n.subscribe(topic + "back", 1000, callbackRos);
    ros::Rate loop_rate(10.0);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    capnzero_eval::EvalMessageRos msg;
    uint32_t msgCounter = 0;
    int payloadBytes = 8;
    std::random_device engine;

    while (ros::ok() && payloadBytes < pow(2, 21)) {

        // fill payload with multiple of 8 bytes
        for (int i = 0; i < payloadBytes / 4; i++) {
            //        for (int i = 0; i < 512; i++) {
            msg.payload.push_back(engine());
        }

        std::cout << "CapnZeroEvaluation::evalRos: Payload Bytes\t" << payloadBytes << "\t Message Size [Bytes]: \t"
                  << ros::serialization::serializationLength(msg) << std::endl;

        while (ros::ok() && msgCounter != 1000) {
            msg.id = ++msgCounter;
            experimentLog->addStartedMeasurement(msgCounter, std::chrono::high_resolution_clock::now());
            pub.publish(msg);
            loop_rate.sleep();
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // log statistics
        experimentLog->calcStatistics();
        experimentLog->serialise(std::to_string(ros::serialization::serializationLength(msg)));

        // reset stuff and increase payload
        experimentLog->reset();
        msgCounter = 0;
        msg.payload.clear();
        payloadBytes *= 2;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg)
{
    experimentLog->finishMeasurement(msg->id, std::chrono::high_resolution_clock::now());
}

void evalCapnZero(std::string topic)
{
    experimentLog = new ExperimentLog("results", "UDP");

    void* ctx = zmq_ctx_new();
    capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::UDP);
    pub->addAddress("224.0.0.2:5500");
//            capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::IPC);
//            pub->addAddress("@capnzeroSend.ipc");
//        capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::TCP);
//        pub->addAddress("141.51.122.36:5500");

    pub->setDefaultTopic(topic);

    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::UDP);
    sub->addAddress("224.0.0.2:5554");
//            capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::IPC);
//            sub->addAddress("@capnzeroReceive.ipc");
//        capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::TCP);
//        sub->addAddress("141.51.122.62:5554");

    sub->setTopic(topic);
    sub->subscribe(&callbackCapnZero);

    // Cap'n Zero: create proto message
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero_eval::EvalMessageCapnZero::Builder msg = msgBuilder.initRoot<capnzero_eval::EvalMessageCapnZero>();
    uint32_t msgCounter = 0;
    int payloadBytes = 8;
    std::random_device engine;

    while (!interrupted && payloadBytes < pow(2, 21)) {
        // fill payload with multiple of 4 bytes
        ::capnp::List<::uint32_t>::Builder payloadBuilder = msg.initPayload(payloadBytes);
        for (int i = 0; i < payloadBytes; i++) {
            payloadBuilder.set(i, engine());
        }

        long bytesSent = 0;
        while (!interrupted && msgCounter != 1000) {
            msg.setId(++msgCounter);
            experimentLog->addStartedMeasurement(msgCounter, std::chrono::high_resolution_clock::now());
            bytesSent = pub->send(msgBuilder);
//            std::cout << "CapnZeroEvaluation::evalCapnZero: Bytes Sent\t" << bytesSent << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "CapnZeroEvaluation::evalCapnZero: Bytes Sent\t" << bytesSent << " Payload Bytes\t" << payloadBytes << "\t Message Size [Bytes]: \t"
                  << std::to_string(msg.totalSize().wordCount * 8) << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // log statistics
        experimentLog->calcStatistics();
        experimentLog->serialise(std::to_string(msg.totalSize().wordCount*8));

        // reset stuff and increase payload
        experimentLog->reset();
        msgCounter = 0;
        payloadBytes *= 2;
    }

    delete sub;
    delete pub;
    zmq_ctx_term(ctx);
}

void callbackCapnZero(::capnp::FlatArrayMessageReader& reader)
{
    experimentLog->finishMeasurement(reader.getRoot<capnzero_eval::EvalMessageCapnZero>().getId(), std::chrono::high_resolution_clock::now());
}

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
#include <signal.h>
#include <string>
#include <thread>
#include <random>

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

void evalCapnZero(std::string topic, std::string payload);
void callbackCapnZero(::capnp::FlatArrayMessageReader& reader);

void evalRos(int argc, char** argv, std::string topic);
void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg);

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

    if (std::string("ros").compare(argv[1]) == 0) {
        evalRos(argc, argv, argv[2]);
    } else {
        evalCapnZero(argv[2], argv[3]);
    }

    // calculate and write results to file


    std::cout << "Cleaning up now. " << std::endl;
    delete experimentLog;

    return 0;
}

void evalRos(int argc, char** argv, std::string topic)
{
    ros::init(argc, argv, "evalRos");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<capnzero_eval::EvalMessageRos>(topic, 1000);
    ros::Subscriber sub = n.subscribe(topic + "back", 1000, callbackRos);
    ros::Rate loop_rate(100.0);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    capnzero_eval::EvalMessageRos msg;

    experimentLog = new ExperimentLog("results", "ROS");
    int16_t msgCounter = 0;
    int payloadBytes = 8;
    std::random_device engine;

//    while (ros::ok() && payloadBytes < pow(2,17)) {

        // fill payload with multiple of 8 bytes
//        for (int i = 0; i < payloadBytes/4; i++) {
        for (int i = 0; i < 512; i++) {
            msg.payload.push_back(engine());
        }

        std::cout << "CapnZeroEvaluation::callbackRos: Payload Bytes\t" << payloadBytes << "\t Message Size [Bytes]: \t" << ros::serialization::serializationLength(msg) << std::endl;

        while (ros::ok() && msgCounter != 1000) {
            msg.id = ++msgCounter;
            experimentLog->addStartedMeasurement(msgCounter, std::chrono::high_resolution_clock::now());
            pub.publish(msg);
            ros::spinOnce();
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
//    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg)
{
    experimentLog->finishMeasurement(msg->id, std::chrono::high_resolution_clock::now());
}

void evalCapnZero(std::string topic, std::string payload)
{
    void* ctx = zmq_ctx_new();
    //        capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::UDP);
    //        pub->addAddress("224.0.0.2:5500");
    //        capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::IPC);
    //        pub->addAddress("@capnzeroSend.ipc");
    capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::TCP);
    pub->addAddress("127.0.0.1:5500");

    pub->setDefaultTopic(topic);

    //        capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::UDP);
    //        sub->addAddress("224.0.0.2:5554");
    //        capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::IPC);
    //        sub->addAddress("@capnzeroReceive.ipc");
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::TCP);
    sub->addAddress("127.0.0.1:5554");

    sub->setTopic(topic);
    sub->subscribe(&callbackCapnZero);

    // Cap'n Zero: create proto message
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero_eval::EvalMessage::Builder beaconMsgBuilder = msgBuilder.initRoot<capnzero_eval::EvalMessage>();
    beaconMsgBuilder.setPayload(payload);

    experimentLog = new ExperimentLog("results", "TCP");
    int16_t counter = 0;
    while (!interrupted && counter != 1000) {
        beaconMsgBuilder.setId(++counter);
        experimentLog->addStartedMeasurement(counter, std::chrono::high_resolution_clock::now());
        pub->send(msgBuilder);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    delete sub;
    delete pub;
    zmq_ctx_term(ctx);
}

void callbackCapnZero(::capnp::FlatArrayMessageReader& reader)
{
    experimentLog->finishMeasurement(reader.getRoot<capnzero_eval::EvalMessage>().getId(), std::chrono::high_resolution_clock::now());
}

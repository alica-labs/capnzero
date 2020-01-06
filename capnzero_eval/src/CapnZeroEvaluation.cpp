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

#include <rosmq/Publisher.h>
#include <rosmq/Subscriber.h>

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

void evalRosMQ(std::string topic);
void callbackRosMQ(capnzero_eval::EvalMessageRos& msg);

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
    } else if (std::string("capnzero").compare(argv[1]) == 0) {
        evalCapnZero(argv[2]);
    } else {
        evalRosMQ(argv[2]);
    }

    std::cout << "Cleaning up now. " << std::endl;
    delete experimentLog;

    return 0;
}

void evalRos(int argc, char** argv, std::string topic)
{
    experimentLog = new ExperimentLog("results", "ROS-UDP/33");

    ros::init(argc, argv, "evalRos");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<capnzero_eval::EvalMessageRos>(topic, 2000);
    ros::Subscriber sub = n.subscribe(topic + "back", 2000, callbackRos, ros::TransportHints().udp());
    ros::Rate loop_rate(33.0);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    capnzero_eval::EvalMessageRos msg;
    uint32_t msgCounter = 0;
    int numInts = 1;
    std::random_device engine;
//    int interval;

    while (ros::ok() && numInts < pow(2, 21)) {
//        interval = 5;
//        while (interval <= 100) {
            for (int i = 0; i < numInts; i++) {
                msg.payload.push_back(engine());
            }

            while (ros::ok() && msgCounter != 1000) {
                msg.id = ++msgCounter;
                experimentLog->addStartedMeasurement(msgCounter, std::chrono::high_resolution_clock::now());
                pub.publish(msg);
//                std::this_thread::sleep_for(std::chrono::milliseconds(interval));
                loop_rate.sleep();
            }

            std::this_thread::sleep_for(std::chrono::seconds(5));

            std::cout << "CapnZeroEvaluation::evalRos: Number of Int32\t" << numInts + 1
                      << "\t Serialised Message Size [Bytes]: \t"
                      << ros::serialization::serializationLength(msg) << std::endl;

            // log statistics
            experimentLog->calcStatistics();
//            experimentLog->serialise(std::to_string(1000.0/interval) + "\t" + std::to_string(numInts + 1) );
            experimentLog->serialise(std::to_string(numInts + 1) );


            // reset stuff and increase payload
            experimentLog->reset();
            msgCounter = 0;
            msg.payload.clear();
//            interval += 5;
//        }
        numInts *= 2;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg)
{
    experimentLog->finishMeasurement(msg->id, std::chrono::high_resolution_clock::now());
}

void evalRosMQ(std::string topic)
{
    experimentLog = new ExperimentLog("results", "ROSMQ-TCP");

    void* ctx = zmq_ctx_new();
    //    rosmq::Publisher* pub = new rosmq::Publisher(ctx, rosmq::Protocol::UDP;
    //    pub->addAddress("224.0.0.2:5500");
    rosmq::Publisher* pub = new rosmq::Publisher(ctx, rosmq::Protocol::IPC);
    pub->addAddress("@capnzeroSend.ipc");
    //    rosmq::Publisher* pub = new rosmq::Publisher(ctx, rosmq::Protocol::TCP);
    //        pub->addAddress("141.51.122.36:5500");
    //    pub->addAddress("127.0.0.1:5500");

    pub->setDefaultTopic(topic);

    //    rosmq::Subscriber<capnzero_eval::EvalMessageRos>* sub = new rosmq::Subscriber<capnzero_eval::EvalMessageRos>(ctx, rosmq::Protocol::UDP);
    //    sub->addAddress("224.0.0.2:5554");
    rosmq::Subscriber<capnzero_eval::EvalMessageRos>* sub = new rosmq::Subscriber<capnzero_eval::EvalMessageRos>(ctx, rosmq::Protocol::IPC);
    sub->addAddress("@capnzeroReceive.ipc");
    //    rosmq::Subscriber<capnzero_eval::EvalMessageRos>* sub = new rosmq::Subscriber<capnzero_eval::EvalMessageRos>(ctx, rosmq::Protocol::TCP);
    //        sub->addAddress("141.51.122.62:5554");
    //    sub->addAddress("127.0.0.1:5554");

    sub->setTopic(topic);
    sub->subscribe(&callbackRosMQ);

    // Cap'n Zero: create proto message
    capnzero_eval::EvalMessageRos msg;
    uint32_t msgCounter = 0;
    int numInts = pow(2, 21);
    std::random_device engine;

    while (!interrupted && numInts < pow(2, 24)) {
        for (int i = 0; i < numInts; i++) {
            msg.payload.push_back(engine());
        }

        long bytesSent = 0;
        while (!interrupted && msgCounter != 1000) {
            msg.id = ++msgCounter;
            experimentLog->addStartedMeasurement(msgCounter, std::chrono::high_resolution_clock::now());
            bytesSent = pub->send(&msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "CapnZeroEvaluation::evalRosMQ: Bytes Sent\t" << bytesSent << " Number of Int32\t" << numInts + 1
                  << "\t Serialised Message Size [Bytes]: \t" << ros::serialization::serializationLength(msg) << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(5));

        // log statistics
        experimentLog->calcStatistics();
        experimentLog->serialise(std::to_string(numInts + 1));

        // reset stuff and increase payload
        experimentLog->reset();
        msgCounter = 0;
        numInts *= 2;
    }

    delete sub;
    delete pub;
    zmq_ctx_term(ctx);
}

void callbackRosMQ(capnzero_eval::EvalMessageRos& msg)
{
    experimentLog->finishMeasurement(msg.id, std::chrono::high_resolution_clock::now());
}

void evalCapnZero(std::string topic)
{
    experimentLog = new ExperimentLog("results", "CAPNZERO-TCP/33");

    void* ctx = zmq_ctx_new();
//    capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::UDP);
//    pub->addAddress("224.0.0.2:5500");
//    capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::IPC);
//    pub->addAddress("@capnzeroSend.ipc");
            capnzero::Publisher* pub = new capnzero::Publisher(ctx, capnzero::Protocol::TCP);
            pub->addAddress("192.168.178.52:5500");

    pub->setSendQueueSize(2000);
    pub->setDefaultTopic(topic);

//        capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::UDP);
//        sub->addAddress("224.0.0.2:5554");
//    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::IPC);
//    sub->addAddress("@capnzeroReceive.ipc");
            capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::TCP);
            sub->addAddress("192.168.178.52:5554");

    sub->setTopic(topic);
    sub->setReceiveQueueSize(2000);
    sub->subscribe(&callbackCapnZero);

    // Cap'n Zero: create proto message
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero_eval::EvalMessageCapnZero::Builder msg = msgBuilder.initRoot<capnzero_eval::EvalMessageCapnZero>();
    uint32_t msgCounter = 0;
    int numInts = 1;
    std::random_device engine;

    while (!interrupted && numInts < pow(2, 21)) {
        // fill payload with multiple of 4 bytes
        ::capnp::List<::uint32_t>::Builder payloadBuilder = msg.initPayload(numInts);
        for (int i = 0; i < numInts; i++) {
            payloadBuilder.set(i, engine());
        }

        long bytesSent = 0;
        while (!interrupted && msgCounter != 1000) {
            msg.setId(++msgCounter);
            experimentLog->addStartedMeasurement(msgCounter, std::chrono::high_resolution_clock::now());
            bytesSent = pub->send(msgBuilder);
            //            std::cout << "CapnZeroEvaluation::evalCapnZero: Bytes Sent\t" << bytesSent << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }

        std::cout << "CapnZeroEvaluation::evalCapnZero: Bytes Sent\t" << bytesSent << " Number of Int32\t" << numInts + 1
                  << "\t Serialised Message Size [Bytes]: \t" << std::to_string(msg.totalSize().wordCount * 8) << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(5));

        // log statistics
        experimentLog->calcStatistics();
        experimentLog->serialise(std::to_string(numInts + 1));

        // reset stuff and increase payload
        experimentLog->reset();
        msgCounter = 0;
        numInts *= 2;
    }

    delete sub;
    delete pub;
    zmq_ctx_term(ctx);
}

void callbackCapnZero(::capnp::FlatArrayMessageReader& reader)
{
    experimentLog->finishMeasurement(reader.getRoot<capnzero_eval::EvalMessageCapnZero>().getId(), std::chrono::high_resolution_clock::now());
}

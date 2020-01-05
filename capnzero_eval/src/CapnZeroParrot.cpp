#include "capnzero-eval-msgs/EvalMessageCapnZero.capnp.h"
#include "capnzero_eval/EvalMessageRos.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnzero/Common.h>
#include <capnzero/Publisher.h>
#include <capnzero/Subscriber.h>
#include <kj/array.h>
#include <signal.h>

#include <rosmq/Publisher.h>
#include <rosmq/Subscriber.h>

#include <ros/ros.h>

#include <vector>

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

void evalRosMQ(std::string topic);
void callbackRosMQ(capnzero_eval::EvalMessageRos& msg);
rosmq::Publisher* pubRosMQ;

void evalRos(int argc, char** argv, std::string topic);
void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg);
ros::Publisher pubRos;

void callbackCapnZero(::capnp::FlatArrayMessageReader& reader);
void evalCapnZero(std::string topic);
capnzero::Publisher* pubCapnZero;

int main(int argc, char** argv)
{
    s_catch_signals();

    if (argc <= 1) {
        std::cerr << "Synopsis: rosrun capnzero_eval Parrot [ros|capnzero] <Topic>" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    if (std::string("ros").compare(argv[1]) == 0) {
        evalRos(argc, argv, argv[2]);
    } else if (std::string("capnzero").compare(argv[1]) == 0){
        evalCapnZero(argv[2]);
    } else {
        evalRosMQ(argv[2]);
    }
}

void evalRosMQ(std::string topic)
{
    void* ctx = zmq_ctx_new();
//    rosmq::Subscriber<capnzero_eval::EvalMessageRos>* sub = new rosmq::Subscriber<capnzero_eval::EvalMessageRos>(ctx, rosmq::Protocol::UDP);
//    sub->addAddress("224.0.0.2:5500");
    rosmq::Subscriber<capnzero_eval::EvalMessageRos>* sub = new rosmq::Subscriber<capnzero_eval::EvalMessageRos>(ctx, rosmq::Protocol::IPC);
        sub->addAddress("@capnzeroSend.ipc");
//    rosmq::Subscriber<capnzero_eval::EvalMessageRos>* sub = new rosmq::Subscriber<capnzero_eval::EvalMessageRos>(ctx, rosmq::Protocol::TCP);
//    sub->addAddress("141.51.122.36:5500");
//    sub->addAddress("127.0.0.1:5500");

    sub->setTopic(topic);
    sub->subscribe(&callbackRosMQ);

//    pubRosMQ = new rosmq::Publisher(ctx, rosmq::Protocol::UDP);
//    pubRosMQ->addAddress("224.0.0.2:5554");
    pubRosMQ = new rosmq::Publisher(ctx, rosmq::Protocol::IPC);
    pubRosMQ->addAddress("@capnzeroReceive.ipc");
//    pubRosMQ = new rosmq::Publisher(ctx, rosmq::Protocol::TCP);
//    pubRosMQ->addAddress("141.51.122.62:5554");
//    pubRosMQ->addAddress("127.0.0.1:5554");

    pubRosMQ->setDefaultTopic(topic);

    while (!interrupted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    delete sub;
    delete pubRosMQ;
    zmq_ctx_term(ctx);
}

void callbackRosMQ(capnzero_eval::EvalMessageRos& msg)
{
    pubRosMQ->send(&msg);
}

void evalRos(int argc, char** argv, std::string topic)
{
    ros::init(argc, argv, "Parrot");
    ros::NodeHandle n;
    pubRos = n.advertise<capnzero_eval::EvalMessageRos>(topic+"back", 2000);
    ros::Subscriber sub = n.subscribe(topic, 2000, callbackRos, ros::TransportHints().tcp().tcpNoDelay(true));
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        loop_rate.sleep();
    }
}

void callbackRos(const capnzero_eval::EvalMessageRos::ConstPtr& msg)
{
    pubRos.publish(msg);
}

void evalCapnZero(std::string topic)
{
    void* ctx = zmq_ctx_new();
//    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::UDP);
//    sub->addAddress("224.0.0.2:5500");
//    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::IPC);
//    sub->addAddress("@capnzeroSend.ipc");
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, capnzero::Protocol::TCP);
    sub->addAddress("192.168.178.49:5500");

    sub->setReceiveQueueSize(2000);
    sub->setTopic(topic);
    sub->subscribe(&callbackCapnZero);

//    pubCapnZero = new capnzero::Publisher(ctx, capnzero::Protocol::UDP);
//    pubCapnZero->addAddress("224.0.0.2:5554");
//    pubCapnZero = new capnzero::Publisher(ctx, capnzero::Protocol::IPC);
//    pubCapnZero->addAddress("@capnzeroReceive.ipc");
    pubCapnZero = new capnzero::Publisher(ctx, capnzero::Protocol::TCP);
    pubCapnZero->addAddress("192.168.178.49:5554");

    pubCapnZero->setSendQueueSize(2000);
    pubCapnZero->setDefaultTopic(topic);

    while (!interrupted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    delete sub;
    delete pubCapnZero;
    zmq_ctx_term(ctx);
}

void callbackCapnZero(::capnp::FlatArrayMessageReader& reader)
{
    ::capnp::MallocMessageBuilder msgBuilder;
    //    capnzero_eval::EvalMessage::Builder msg = msgBuilder.initRoot<capnzero_eval::EvalMessage>();
    //    msg.setPayload(reader.getRoot<capnzero_eval::EvalMessage>().getPayload());
    //    msg.setId(reader.getRoot<capnzero_eval::EvalMessage>().getId());
    msgBuilder.setRoot<capnzero_eval::EvalMessageCapnZero::Reader>(reader.getRoot<capnzero_eval::EvalMessageCapnZero>());
    pubCapnZero->send(msgBuilder);
}
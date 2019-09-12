#include <rosmq/Publisher.h>
#include <rosmq/Common.h>
#include <std_msgs/String.h>
#include <chrono>
#include <thread>
#include <signal.h>

//#define DEBUG_PUB

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
        std::cerr << "Synopsis: rosrun rosmq pub \"String that should be published!\"" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    // ROS msg
    std_msgs::String msg;
    msg.data = "asdf";

#ifdef DEBUG_PUB
    std::cout << "pub: Message to send: " << msg << std::endl;
#endif

    void* ctx = zmq_ctx_new();
    rosmq::Publisher pub = rosmq::Publisher(ctx, rosmq::Protocol::UDP);
    pub.setDefaultTopic(argv[1]);
//    pub.addAddress("@rosmq.ipc");
    pub.addAddress("224.0.0.2:5555");
//    pub.addAddress("127.0.0.1:5555");
    while (!interrupted) {
        int numBytesSent = pub.send(&msg);
#ifdef DEBUG_PUB
        std::cout << "pub: " << numBytesSent << " Bytes sent!" << std::endl;
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // wait until everything is send
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

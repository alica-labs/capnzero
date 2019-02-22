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

struct argvVar
{
    const char *name;    // the name of the variable to look for
    const char **value;  // the address of the variable to set the value
};
int16_t l;

std::string s0, s3;

#define DEBUG_SENDER

int16_t j;

void callback(::capnp::FlatArrayMessageReader& k);



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
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, argv[1]);

    capnzero::Publisher pub = capnzero::Publisher(ctx, argv[1]);
//    sub->connect(capnzero::CommType::IPC, "@capnzero.ipc");
    sub->connect(capnzero::CommType::UDP, "224.0.0.2:5500");
//    sub->connect(capnzero::CommType::TCP, "141.51.122.62:5555");
    sub->subscribe(&callback);




   // init builder
    ::capnp::MallocMessageBuilder s1;

  //  s3="hi";
    capnproto::Capnprotoperformancetest::Builder lu= s1.initRoot<capnproto::Capnprotoperformancetest>();

   // std::cout<<"Data inside my strin lu: "<<s1.getRoot<capnzero::String>().toString().flatten().cStr()<<std::endl;
    //Publisher  part

//    pub.bind(capnzero::CommType::IPC, "@capnzero.ipc");
      pub.bind(capnzero::CommType::UDP, "224.0.0.2:5554");
   // pub.bind(capnzero::CommType::TCP, "141.51.122.62:5555");
  // pub.bind("tcp://*:5563")

    while (!interrupted) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::this_thread::sleep_for(std::chrono::seconds(2));

        int numBytesSent = pub.send(s1);
        {
            std::cout << "I am going to publish the following message : "<< numBytesSent << " Bytes sent!" << std::endl;
           // std::cout<<"Data inside my strin lu: "<<s0 /*.getRoot<capnzero::String>().toString().flatten().cStr()*/<<std::endl;
           lu.setString(s3);
           lu.setNumber(j);

            //std::cout << "pub: Message to send:lu " <<lu.toString().flatten().cStr() << std::endl;
            s0.clear();
            s3.clear();
            lu.hasString();
            //lu.disownString();

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

/*#ifdef DEBUG_PUB
            std::cout << "I am going to publish the following message : "<< numBytesSent << " Bytes sent!" << std::endl;
#endif
  */
        //std::this_thread::sleep_for(std::chrono::seconds(3));
    }// wait until everything is send
    //std::this_thread::sleep_for(std::chrono::seconds(1));
    //{
    //  std::this_thread::sleep_for(std::chrono::milliseconds(500));



    //std::this_thread::sleep_for(std::chrono::milliseconds(500));


delete sub;

    zmq_ctx_term(ctx);


}

void callback(::capnp::FlatArrayMessageReader& k) {

    std::cout << "subscriber called for port 5500 and rcvd message ...." << std::endl;
    k.getRoot<capnproto::Capnprotoperformancetest>().toString().flatten().cStr();

    s3=k.getRoot<capnproto::Capnprotoperformancetest>().getString();

    //std::cout << k.getRoot<capnzero::Capnprotoperformancetest>().getNumber()<<std::endl;//
    // .toString().flatten().cStr() << std::endl;
    j=int16_t (k.getRoot<capnproto::Capnprotoperformancetest>().getNumber());
    s0 = k.getRoot<capnproto::Capnprotoperformancetest>().toString().flatten().cStr();
    //=s0[2];
    // auto [s0.size()]=s0.data();

    std::cout << "Called callback..."<<s3 << std::endl;
/*
    ::capnp::MallocMessageBuilder s1;

    capnzero::String::Builder lu=s1.initRoot<capnzero::String>();
    lu.setString(s0);{
    std::cout << "Called callback..." << s0<<std::endl;

    void *ctx = zmq_ctx_new();
    capnzero::Publisher pub = capnzero::Publisher(ctx,);
//    pub.bind(capnzero::CommType::IPC, "@capnzero.ipc");
    pub.bind(capnzero::CommType::UDP, "223.0.0.2:5554");


    // pub.bind(capnzero::CommType::TCP, "141.51.122.62:5555");
    pub.send(s0);

*/

}
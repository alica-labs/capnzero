#include "capnzero-base-msgs/string.capnp.h"
#include <capnzero/Publisher.h>
#include <capnzero/Common.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <capnzero/Subscriber.h>
#include <string>
#include <iostream>
#include <map>
//#define DEBUG_PUB
int16_t k=0;
std::map<int16_t , std::chrono::time_point<std::chrono::high_resolution_clock >> measuringMap;
std::string leo,Scorpio;

static bool interrupted = false;
static void s_signal_handler(int signal_value)
{
    interrupted = true;
}
void callback(::capnp::FlatArrayMessageReader& reader);

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
        std::cerr << "Synopsis: rosrun capnzero pub \"String that should be published!\"" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    // Cap'n Proto: create proto message

    // init builder
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero::Capnprotoperformancetest::Builder beaconMsgBuilder = msgBuilder.initRoot<capnzero::Capnprotoperformancetest>();

    // set content
    beaconMsgBuilder.setString(argv[2]);




#ifdef DEBUG_PUB
    std::cout << "pub: Message to send: " << beaconMsgBuilder.toString().flatten().cStr() << std::endl;
#endif
    void* ctx = zmq_ctx_new();


//Publisher  part

    capnzero::Publisher pub = capnzero::Publisher(ctx, argv[1]);
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, argv[1]);
//    pub.bind(capnzero::CommType::IPC, "@capnzero.ipc");
    pub.bind(capnzero::CommType::UDP, "224.0.0.2:5500");
//    pub.bind(capnzero::CommType::TCP, "141.51.122.62:5555");


//Subscriber part

    //capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, argv[1]);
    //sub->connect(capnzero::CommType::IPC, "@capnzero.ipc");
   sub->connect(capnzero::CommType::UDP, "224.0.0.2:5554");
   // sub->connect(capnzero::CommType::TCP, "141.51.122.62:5555");
    sub->subscribe(&callback);


    while (!interrupted)
    {
        {     beaconMsgBuilder.setNumber(k);
            int numBytesSent = pub.send(msgBuilder);
           // measuringMap.emplace(msgBuilder, std::chrono::high_resolution_clock::now());
            measuringMap.emplace(k, std::chrono::high_resolution_clock::now());
            {
                std::cout << "Master is going to publish : "<< numBytesSent << " Bytes sent!" << std::endl;}
/*#ifdef DEBUG_PUB
            std::cout << "I am going to publish the following message : "<< numBytesSent << " Bytes sent!" << std::endl;
#endif
  */
   std::this_thread::sleep_for(std::chrono::seconds(2));
k++;
        }// wait until everything is send
        std::this_thread::sleep_for(std::chrono::seconds(2));
        //{
          //  std::this_thread::sleep_for(std::chrono::milliseconds(500));



        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    //sleep(2);
    //std::this_thread::sleep_for(std::chrono::seconds(2));


    std::cout << "We must have missed " << measuringMap.size() << " Number of Msgs!" <<std::endl;
    for (auto& entry : measuringMap)
    {
        std::cout << "ID: " << entry.first << " StartTime: " << std::chrono::duration_cast<std::chrono::milliseconds>(entry.second.time_since_epoch()).count() << std::endl;
    }
//}
    delete sub;

    zmq_ctx_term(ctx);
    return 0;
}


void callback(::capnp::FlatArrayMessageReader& reader)

{
    // auto k=reader<capnzero::Capnprotoperformancetest::Builder>;


    //auto K=reader.getOptions();

    //capnzero::Capnprotoperformancetest::Builder beaconMsgBuilder = reader;
//auto j= reader.getOptions<capnzero::Capnprotoperformancetest>().nestingLimit;
    std::cout << "I have recievd the following from the Subscriber" << std::endl;
    std::cout << reader.getRoot<capnzero::Capnprotoperformancetest>().toString().flatten().cStr() << std::endl;
    leo = reader.getRoot<capnzero::Capnprotoperformancetest>().toString().flatten().cStr() ;
    k=reader.getRoot<capnzero::Capnprotoperformancetest>().getNumber();
    Scorpio=reader.getRoot<capnzero::Capnprotoperformancetest>().getString();
    std::cout << "The size of rcvd str is " << leo.length() << " bytes.\n";

    //std::cout <<"The data inside redaer"<<<<std::endl;
    auto mapEntry = measuringMap.find(k);
    if (mapEntry != measuringMap.end()) {
        std::cout<<"Received ID: " << k << " Time elapsed is: "
                 << std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now() - mapEntry->second).count()<< std::endl;
    }
}
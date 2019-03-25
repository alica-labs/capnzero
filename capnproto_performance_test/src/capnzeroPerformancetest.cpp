#include "capnproto-base-msgs/Testmessage.capnp.h"
#include <capnzero/Publisher.h>
#include <capnzero/Common.h>
#include <capnzero/Subscriber.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <chrono>
#include <thread>
#include <signal.h>

#include <string>
#include <iostream>
#include <map>
#include "Statistics.h"
#include <fstream>
//#include ""
#define DEBUG_PUB //Macros ,Basic C

std::map<int16_t , std::chrono::time_point<std::chrono::high_resolution_clock >> measuringMap;
std::map<long, double >Mymap;
static bool interrupted = false;
static void s_signal_handler(int signal_value)
{
    interrupted = true;
}
void callback(::capnp::FlatArrayMessageReader& reader); //function prototype

static void s_catch_signals(void)
{
    struct sigaction action;
    action.sa_handler = s_signal_handler;
    action.sa_flags = 0;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
}
int main(int argc, char** argv)// Stack frame started
{
    int16_t counter=0;
    s_catch_signals();
    std::ofstream data_saver;
    data_saver.open("result_storage/result.csv");
    data_saver << "Capnzero Measurement summary.\n ";



    if (argc <= 1) {
        std::cerr << "Synopsis: rosrun capnproto pub \"String that should be published!\"" << std::endl;
        return -1;
    }
    for (size_t i = 0; i < argc; i++) {
        std::cout << "Param " << i << ": '" << argv[i] << "'" << std::endl;
    }

    // Cap'n Proto: create proto message
    Statistics<double> *measurement_unit =new Statistics<double > ;
    // init builder
    ::capnp::MallocMessageBuilder msgBuilder;
    capnzero::Testmessage::Builder beaconMsgBuilder = msgBuilder.initRoot<capnzero::Testmessage>();

    // set content
    beaconMsgBuilder.setPayload(argv[2]);
#ifdef DEBUG_PUB
    std::cout << "pub: Message to send: " << beaconMsgBuilder.toString().flatten().cStr() << std::endl;
#endif
    void* ctx = zmq_ctx_new();

//Publisher  part
    capnzero::Publisher * pub = new capnzero::Publisher(ctx, argv[1]);
    capnzero::Subscriber* sub = new capnzero::Subscriber(ctx, argv[1]); // creating a pointer in the heap /free pool of memory
                                                                          //Dynamic memory allocation
    pub->bind(capnzero::CommType::UDP, "224.0.0.2:5500");

//Subscriber part
    sub->connect(capnzero::CommType::UDP, "224.0.0.2:5554");
    sub->subscribe(&callback);

    while (!interrupted)
    {
        {     beaconMsgBuilder.setId(counter);
            int numBytesSent = pub->send(msgBuilder);
            measuringMap.emplace(counter, std::chrono::high_resolution_clock::now());
            {
                std::cout << "Publisher is going to publish: "<< numBytesSent << " Bytes sent!" << std::endl;
                std::cout << "Publisher is going to publish: "<< counter <<" Message" << std::endl;}
#ifdef DEBUG_PUB
            std::cout << "I am going to publish the following message : "<< numBytesSent << " Bytes sent!" << std::endl;
#endif
            //std::this_thread::sleep_for(std::chrono::seconds(2));
        }// wait until everything is send
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        counter++;
    }
    std::cout << "We must have missed: " << measuringMap.size() << " Number of Msgs!" <<std::endl;

    std::cout << "number of rcvd msg: "<<Mymap.size()<<std::endl;
    data_saver<<"We must have missed: " << measuringMap.size() << " Number of Msgs!" <<std::endl;
    data_saver<<"Number of rcvd msg: "<<Mymap.size()<<std::endl ;
    data_saver <<"Missed Messages Id,Start time (ms),Mean time (ms),Standard deviation (ms),Maximum time (ms),Minimum time (ms) \n   ";
    for (auto& entry : measuringMap)
    {
        std::cout << "ID: " << entry.first << " StartTime: " << std::chrono::duration_cast<std::chrono::milliseconds>(entry.second.time_since_epoch()).count() << std::endl;
        data_saver << entry.first <<";" << std::chrono::duration_cast<std::chrono::milliseconds>(entry.second.time_since_epoch()).count() << std::endl;
    }
    auto mEan =measurement_unit->referencemean(Mymap); // access member function using pointer
    auto standard_deviation= measurement_unit->referencestd_dev(Mymap);// access member function using pointer
    auto  mAx = measurement_unit->rmax(Mymap);// access member function using pointer
    auto mIn=measurement_unit->rmin(Mymap);// access member function using pointer
    data_saver <<";"<<";"<< mEan<<";"<<standard_deviation<<";"<<mAx<<";"<< mIn <<std::endl;
    data_saver.close();
    std::cout << "Cleaning up now. "  << std::endl;
    delete  measurement_unit;
    Mymap.clear();
    measuringMap.clear();
    delete sub;
    delete pub;// dynamic memory deallocation
    zmq_ctx_term(ctx);
    return 0;
}

void callback(::capnp::FlatArrayMessageReader& reader) //Stack frame after main call this function using subscriber
                                                       //Function declaration or Body
{
    std::cout << "I have received the following from the subscriber: " << std::endl;
    std::string rcvdmsg = reader.getRoot<capnzero::Testmessage>().toString().flatten().cStr() ;//local variable
    register int16_t rcvmsgnumber=reader.getRoot<capnzero::Testmessage>().getId(); //local variable in register
    register std::string rcvmsgstring=reader.getRoot<capnzero::Testmessage>().getPayload(); //local variable in register
    std::cout << "Size of the received str is: " << rcvdmsg.length() << " \n";
    std::cout << "String data inside received msg: " << rcvmsgstring << " \n";
    std::cout << "Integer data inside received msg: "<< rcvmsgnumber << " \n";
    auto mapEntry = measuringMap.find(rcvmsgnumber);
    if (mapEntry != measuringMap.end()) {
        std::cout<<"Received ID: " << rcvmsgnumber << " Time elapsed is: "
                 << std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now() - mapEntry->second).count()<<" ms"<< std::endl;
        double time_passed=double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - mapEntry->second).count() );
        // here is my Mymap is the Map containner
        Mymap.emplace(rcvmsgnumber,time_passed);
        measuringMap.erase(rcvmsgnumber);
    }
    else
    {
        std::cerr << "That is Strange!" << std::endl;
    }

}
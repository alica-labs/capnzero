#include "discovery/Agent.h"

#include <capnzero/CapnZero.h>

#include <zmq.h>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>

#include <assert.h>
#include <chrono>
#include <iostream>
#include <string.h>
#include <unistd.h>

// Network stuff
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>

#include <capnp/endian.h>

#define DEBUG_AGENT

namespace discovery
{

bool Agent::operating = true;

Agent::Agent(std::string name, bool sender)
        : Worker(name)
        , sender(sender)
        , socket(nullptr)
        , ctx(zmq_ctx_new())
{
    // generate
    uuid_generate(this->uuid);

    if (this->sender) {
        this->pub = new capnzero::Publisher(this->ctx, "MCGroup");
        this->pub->bind(capnzero::CommType::UDP, "udp://224.0.0.1:5555");
    } else {
        this->sub = new capnzero::Subscriber(this->ctx, "MCGroup");
        this->sub->connect(capnzero::CommType::UDP, "udp://224.0.0.1:5555");
        this->sub->subscribe<discovery::Agent>(&Agent::callback, (discovery::Agent*) this);
    }
}

void Agent::callback(::capnp::FlatArrayMessageReader& reader)
{
    std::cout << "Agent::callback(): " << reader.getRoot<discovery_msgs::Beacon>().toString().flatten().cStr() << std::endl;
}

Agent::~Agent()
{
    if (this->sender) {
        delete (this->pub);
    } else {
        delete (this->sub);
    }

    check(zmq_ctx_term(this->ctx), "zmq_ctx_term", true);
}

void Agent::run()
{
    // this->checkZMQVersion();
    // this->testUUIDStuff();

//    if (!this->getWirelessAddress()) {
//        std::cerr << "Agent: No WLAN-Address available! " << std::endl;
//        return;
//    }

    if (this->sender) {
        send();
    }
}

void Agent::send()
{
    // Cap'n Proto: create proto message

    // init builder
    ::capnp::MallocMessageBuilder msgBuilder;
    discovery_msgs::Beacon::Builder beaconMsgBuilder = msgBuilder.initRoot<discovery_msgs::Beacon>();

    // set content
    beaconMsgBuilder.setIp(this->wirelessIpAddress);
    beaconMsgBuilder.setPort(6666);
    beaconMsgBuilder.setUuid(kj::arrayPtr(this->uuid, sizeof(this->uuid)));

#ifdef DEBUG_AGENT
    std::cout << "Agent:send(): Message to send: " << beaconMsgBuilder.toString().flatten().cStr() << std::endl;
#endif

    int numBytesSent = this->pub->send(msgBuilder);

#ifdef DEBUG_AGENT
    std::cout << "Agent::send(): " << numBytesSent << " Bytes sent!" << std::endl;
#endif
}

/**
 * Checks the return code and reports an error if present.
 * If abortIfError is set to true, it also aborts the process.
 */
void Agent::check(int returnCode, std::string methodName, bool abortIfError)
{
    if (returnCode != 0) {
        std::cerr << methodName << " returned: " << errno << " - " << zmq_strerror(errno) << std::endl;
        if (abortIfError)
            assert(returnCode);
    }
}

void Agent::testUUIDStuff()
{
    uuid_generate(this->uuid);

    // unparse (to string)
    char uuid_str[37]; // ex. "1b4e28ba-2fa1-11d2-883f-0016d3cca427" + "\0"
    uuid_unparse_lower(uuid, uuid_str);
    printf("generate uuid=%s\n", uuid_str);

    // parse (from string)
    uuid_t uuid2;
    uuid_parse(uuid_str, uuid2);

    // compare (rv == 0)
    int rv;
    rv = uuid_compare(uuid, uuid2);
    printf("uuid_compare() result=%d\n", rv);

    // compare (rv == 1)
    uuid_t uuid3;
    uuid_parse("1b4e28ba-2fa1-11d2-883f-0016d3cca427", uuid3);
    rv = uuid_compare(uuid, uuid3);
    printf("uuid_compare() result=%d\n", rv);

    // is null? (rv == 0)
    rv = uuid_is_null(uuid);
    printf("uuid_null() result=%d\n", rv);

    // is null? (rv == 1)
    uuid_clear(uuid);
    rv = uuid_is_null(uuid);
    printf("uuid_null() result=%d\n", rv);
}

/**
 * This method should store the IP address of the W-LAN interface, if available.
 * @return True, if IP address is determined, false otherwise.
 */
bool Agent::getWirelessAddress()
{
    struct ifaddrs* ifAddrStruct = NULL;
    struct ifaddrs* ifa = NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        // filters for 'w'ireless interfaces with ipv4 addresses
        if (ifa->ifa_name[0] == 'w' && ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            int error = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), this->wirelessIpAddress, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (error != 0) {
                std::cout << "Agent:getWirelessAddress: getnameinfo() failed: " << gai_strerror(error) << std::endl;
                exit(EXIT_FAILURE);
            }

            if (ifAddrStruct != NULL)
                freeifaddrs(ifAddrStruct);
            return true;
        } else {
            // std::cout << "Agent::getWirelessAddress: Interface Name: " << ifa->ifa_name
            //          << " Protocol Family: " << ifa->ifa_addr->sa_family << std::endl;
        }
    }
    if (ifAddrStruct != NULL)
        freeifaddrs(ifAddrStruct);
    return false;
}

void Agent::checkZMQVersion()
{
    int major, minor, patch;
    zmq_version(&major, &minor, &patch);
    std::cout << "Major: " << major << " Minor: " << minor << " Patch: " << patch << std::endl;
}

void Agent::sigIntHandler(int sig)
{
    operating = false;
}
} // namespace discovery

int main(int argc, char** argv)
{
    discovery::Agent* agent;
    if (argc > 1 && strcmp(argv[1], "sender") == 0) {
        std::cout << "Creating sending agent!" << std::endl;
        agent = new discovery::Agent("Sender", true);
    } else {
        std::cout << "Creating receiving agent!" << std::endl;
        agent = new discovery::Agent("Receiver", false);
    }

    agent->setDelayedStartMS(std::chrono::milliseconds(0));
    agent->setIntervalMS(std::chrono::milliseconds(1000));

    agent->start();

    // register ctrl+c handler
    signal(SIGINT, discovery::Agent::sigIntHandler);

    // start running
    while (discovery::Agent::operating) {
        sleep(1);
    }

    agent->stop();

    delete agent;
    return 0;
}

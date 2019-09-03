#include "capnzero/Subscriber.h"

namespace capnzero
{
const int Subscriber::wordSize = sizeof(capnp::word);

Subscriber::~Subscriber()
{
    this->running = false;
    this->runThread->join();
    delete this->runThread;
    check(zmq_close(this->socket), "zmq_close");
    check(zmq_close(this->udpSocket), "zmq_close");
}

void Subscriber::addAddress(Protocol protocol, std::string addr)
{
    switch (protocol) {
    case Protocol::UDP:
        this->addresses.push_back(Address("udp://" + addr, protocol));
        this->hasUDP = true;
        break;
    case Protocol::TCP:
        this->addresses.push_back(Address("tcp://" + addr, protocol));
        break;
    case Protocol::IPC:
        this->addresses.push_back(Address("ipc://" + addr, protocol));
        break;
    default:
        // Unknown protocol!
        assert(false && "Subscriber::addAddress: Given protocol is unknown!");
    }
}

void Subscriber::connect()
{
    if (this->hasUDP) { // If a udp address was registered the udp socket is initialized
        // Since there is only one topic allowed and only one rcv-timeout you can set them independent of the
        // addresses
        this->udpSocket = zmq_socket(this->context, ZMQ_DISH);
        check(zmq_join(this->udpSocket, this->topic.c_str()), "zmq_join");
        check(zmq_setsockopt(this->udpSocket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
    }
    // Right now this code expects that there is at least one non-UDP address.
    check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
    check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, this->topic.c_str(), 0), "zmq_setsockopt");
    for (int i = 0; i < this->addresses.size(); ++i) {
        Address addr = this->addresses[i];
        switch (addr.type) {
        case Protocol::UDP:
            check(zmq_bind(this->udpSocket, ("udp://" + addr.address).c_str()), "zmq_bind");
            break;
        case Protocol::TCP:
            check(zmq_connect(this->socket, ("tcp://" + addr.address).c_str()), "zmq_connect");
            break;
        case Protocol::IPC:
            check(zmq_connect(this->socket, ("ipc://" + addr.address).c_str()), "zmq_connect");
            break;
        default:
            // Unknown protocol!
            assert(false && "Subscriber::addAddress: Given protocol is unknown!");
        }
    }
    this->isConnected = true;
}

void Subscriber::receive()
{
    while (this->running) {
        zmq_msg_t msg;
        check(zmq_msg_init(&msg), "zmq_msg_init");
        zmq_msg_t topic;
        check(zmq_msg_init(&topic), "zmq_msg_init");
        zmq_msg_recv(&topic, this->socket, 0);
        zmq_msg_t udpMsg;
        check(zmq_msg_init(&udpMsg), "zmq_msg_init");
        int udpNBytes = zmq_msg_recv(&udpMsg, this->udpSocket, 0);
#ifdef DEBUG_SUBSCRIBER
        std::cout << "Subscriber::received() waiting ..." << std::endl;
#endif

        int nbytes = zmq_msg_recv(&msg, this->socket, 0);

//      std::cout << "Subscriber::receive(): nBytes: " << nbytes << " errno: " << errno << "(EAGAIN: " << EAGAIN << ")" << std::endl;

        // handling for unsuccessful call to zmq_msg_recv
        if (nbytes == -1) {
            if (errno != EAGAIN) // receiving a message was unsuccessful
            {
                std::cerr << "Subscriber::receive(): zmq_msg_recv received no bytes! " << errno << " - zmq_strerror(errno)" << std::endl;
            } else // no message available
            {
                if (udpNBytes == -1) { // This is a dirty hack
                    if (errno != EAGAIN) {
                        std::cerr << "Subscriber::receive(): zmq_msg_recv received no bytes! " << errno << " - zmq_strerror(errno)" << std::endl;
                    }
#ifdef DEBUG_SUBSCRIBER
                    //                std::cout << "Subscriber::receive(): continue because of EAGAIN!" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
                } else {
                    msg = udpMsg;
                } // End of hack
            }
#ifdef DEBUG_SUBSCRIBER
            std::cout << ".";
            std::cout.flush();
#endif
            check(zmq_msg_close(&msg), "zmq_msg_close");
            continue;
        }
#ifdef DEBUG_SUBSCRIBER
        else {
            std::cout << std::endl;
        }
#endif

        // Received message must contain an integral number of words.
        if (zmq_msg_size(&msg) % Subscriber::wordSize != 0) {
            std::cout << "Non-Integral number of words!" << std::endl;
            check(zmq_msg_close(&msg), "zmq_msg_close");
            continue;
        }

        // Check whether message is memory aligned
        assert(reinterpret_cast<uintptr_t>(zmq_msg_data(&msg)) % Subscriber::wordSize == 0);

        int numWordsInMsg = zmq_msg_size(&msg);
        auto wordArray = kj::ArrayPtr<capnp::word const>(reinterpret_cast<capnp::word const*>(zmq_msg_data(&msg)), numWordsInMsg);

        ::capnp::FlatArrayMessageReader msgReader = ::capnp::FlatArrayMessageReader(wordArray);

        (this->callbackFunction_)(msgReader);

        check(zmq_msg_close(&msg), "zmq_msg_close");
    }
}
} // namespace capnzero
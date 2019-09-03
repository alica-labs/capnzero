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
}

void Subscriber::addAddress(std::string address)
{
    switch (protocol) {
    case Protocol::UDP:
        this->addresses.push_back(Address("udp://" + address, protocol));
        break;
    case Protocol::TCP:
        this->addresses.push_back(Address("tcp://" + address, protocol));
        break;
    case Protocol::IPC:
        this->addresses.push_back(Address("ipc://" + address, protocol));
        break;
    default:
        // Unknown protocol!
        assert(false && "Subscriber::addAddress: Given protocol is unknown!");
    }
}

void Subscriber::connect()
{
    // Right now this code expects that there is at least one non-UDP address.
    check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
    check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, this->defaultTopic.c_str(), 0), "zmq_setsockopt");
    for (int i = 0; i < this->addresses.size(); ++i) {
        Address addr = this->addresses[i];
        switch (addr.protocol) {
        case Protocol::UDP:
            check(zmq_bind(this->socket, ("udp://" + addr.address).c_str()), "zmq_bind");
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
}

void Subscriber::receive()
{
    while (this->running) {
        zmq_msg_t msg;
        check(zmq_msg_init(&msg), "zmq_msg_init");
        zmq_msg_t topic;
        check(zmq_msg_init(&topic), "zmq_msg_init");
        zmq_msg_recv(&topic, this->socket, 0);
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

            if (errno != EAGAIN) {
                std::cerr << "Subscriber::receive(): zmq_msg_recv received no bytes! " << errno << " - zmq_strerror(errno)" << std::endl;
            }
#ifdef DEBUG_SUBSCRIBER
            else {
                std::cout << "Subscriber::receive(): continue because of EAGAIN!" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
#endif

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
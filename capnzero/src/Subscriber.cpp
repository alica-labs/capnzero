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

void Subscriber::connect(CommType commType, std::string address)
{
    switch (commType) {
    case CommType::UDP:
        this->socket = zmq_socket(this->context, ZMQ_DISH);
        check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
        std::cout << "Group: " << this->groupName << std::endl;
        check(zmq_join(this->socket, this->groupName.c_str()), "zmq_join");
        check(zmq_bind(this->socket, ("udp://" + address).c_str()), "zmq_bind");
        break;
    case CommType::TCP:
        this->socket = zmq_socket(this->context, ZMQ_SUB);
        check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
        check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, "", 0), "zmq_setsockopt");
        check(zmq_connect(this->socket, ("tcp://" + address).c_str()), "zmq_connect");
        break;
    case CommType::IPC:
        this->socket = zmq_socket(this->context, ZMQ_SUB);
        check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
        check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, "", 0), "zmq_setsockopt");
        check(zmq_connect(this->socket, ("ipc://" + address).c_str()), "zmq_connect");
        break;
    default:
        // Unknown communication type!
        assert(false);
    }
    this->type = commType;
}

void Subscriber::subscribe(void (*callbackFunction)(::capnp::FlatArrayMessageReader&))
{
    this->callbackFunction_ = callbackFunction;
    this->runThread = new std::thread(&Subscriber::receive, this);
}

void Subscriber::receive()
{
    while (this->running) {
        zmq_msg_t msg;
        check(zmq_msg_init(&msg), "zmq_msg_init");
        if (this->type != capnzero::CommType::UDP) {
            zmq_msg_t topic;
            check(zmq_msg_init(&topic), "zmq_msg_init");
            zmq_msg_recv(&topic, this->socket, 0);
        }
#ifdef DEBUG_SUBSCRIBER
        std::cout << "Subscriber::received() waiting ..." << std::endl;
#endif

        int nbytes = zmq_msg_recv(&msg, this->socket, 0);

        //        std::cout << "Subscriber::receive(): nBytes: " << nbytes << " errno: " << errno << "(EAGAIN: " << EAGAIN << ")" << std::endl;

        // handling for unsuccessful call to zmq_msg_recv
        if (nbytes == -1) {
            if (errno != EAGAIN) // receiving a message was unsuccessful
            {
                std::cerr << "Subscriber::receive(): zmq_msg_recv received no bytes! " << errno << " - zmq_strerror(errno)" << std::endl;
            }
#ifdef DEBUG_SUBSCRIBER
            else // no message available
            {
                //                std::cout << "Subscriber::receive(): continue because of EAGAIN!" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
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
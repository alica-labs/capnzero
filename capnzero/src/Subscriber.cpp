#include "capnzero/Subscriber.h"

namespace capnzero
{
const int Subscriber::WORD_SIZE = sizeof(capnp::word);

Subscriber::Subscriber(void* context, Protocol protocol)
        : socket(nullptr)
        , rcvTimeout(500)
        , topic("")
        , context(context)
        , protocol(protocol)
        , running(false)
        , runThread(nullptr)
        , callbackFunction_(nullptr)
{
    // set socket type with respect to given protocol
    switch (this->protocol) {
        case Protocol::UDP:
            this->socket = zmq_socket(this->context, ZMQ_DISH);
            break;
        case Protocol::TCP:
        case Protocol::IPC:
            this->socket = zmq_socket(this->context, ZMQ_SUB);
            break;
        default:
            // Unknown protocol!
            assert(false && "Subscriber::Subscriber: The given protocol is unknown!");
    }

    check(zmq_setsockopt(this->socket, ZMQ_RCVTIMEO, &rcvTimeout, sizeof(rcvTimeout)), "zmq_setsockopt");
}

Subscriber::~Subscriber()
{
    this->running = false;
    this->runThread->join();
    delete this->runThread;
    check(zmq_close(this->socket), "zmq_close");
}

void Subscriber::setTopic(std::string topic)
{
    assert(topic.length() < MAX_TOPIC_LENGTH && "Subscriber::setTopic: The given topic is too long!");

    switch (this->protocol) {
        case Protocol::UDP:
            check(zmq_join(this->socket, this->topic.c_str()), "zmq_join");
            break;
        case Protocol::TCP:
        case Protocol::IPC:
            check(zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, this->topic.c_str(), 0), "zmq_setsockopt");
            break;
        default:
            // Unknown protocol!
            assert(false && "Subscriber::setTopic: The given protocol is unknown!");
    }
    this->topic = topic;
}

void Subscriber::addAddress(std::string address)
{
    switch (protocol) {
    case Protocol::UDP:
        check(zmq_bind(this->socket, ("udp://" + address).c_str()), "zmq_bind");
        break;
    case Protocol::TCP:
        check(zmq_connect(this->socket, ("tcp://" + address).c_str()), "zmq_connect");
        break;
    case Protocol::IPC:
        check(zmq_connect(this->socket, ("ipc://" + address).c_str()), "zmq_connect");
        break;
    default:
        // Unknown protocol!
        assert(false && "Subscriber::addAddress: Given protocol is unknown!");
    }
}

void Subscriber::subscribe(void (*callbackFunction)(::capnp::FlatArrayMessageReader&)) {
    this->callbackFunction_ = callbackFunction;
    if (!running) {
        this->running = true;
        this->runThread = new std::thread(&Subscriber::receive, this);
    }
}

void Subscriber::receive()
{
    while (this->running) {

#ifdef DEBUG_SUBSCRIBER
        std::cout << "Subscriber::received() waiting ..." << std::endl;
#endif

        if (this->protocol != Protocol::UDP) {
            // Topic handling for non-udp protocols via multipart message
            zmq_msg_t topicMsg;
            check(zmq_msg_init(&topicMsg), "zmq_msg_init");
            if (0 == checkReceive(zmq_msg_recv(&topicMsg, this->socket, ZMQ_SNDMORE), topicMsg, "Subscriber::receive-Topic")) {
                // error or timeout on recv
                continue;
            }
        }

        zmq_msg_t msg;
        check(zmq_msg_init(&msg), "zmq_msg_init");
        if (0 == checkReceive(zmq_msg_recv(&msg, this->socket, 0), msg, "Subscriber::receive")) {
            // error or timeout on recv
            continue;
        }

#ifdef DEBUG_SUBSCRIBER
        std::cout << std::endl;
#endif

        // Received message must contain an integral number of words.
        if (zmq_msg_size(&msg) % Subscriber::WORD_SIZE != 0) {
            std::cerr << "Subscriber::receive(): Message received with a size of non-integral number of words!" << std::endl;
            check(zmq_msg_close(&msg), "zmq_msg_close");
            continue;
        }

        // Check whether message is memory aligned
        assert(reinterpret_cast<uintptr_t>(zmq_msg_data(&msg)) % Subscriber::WORD_SIZE == 0);

        // Call the callback with Cap'n Proto message
        int msgSize = zmq_msg_size(&msg);
        auto wordArray = kj::ArrayPtr<capnp::word const>(reinterpret_cast<capnp::word const*>(zmq_msg_data(&msg)), msgSize);
        ::capnp::FlatArrayMessageReader msgReader = ::capnp::FlatArrayMessageReader(wordArray);
        (this->callbackFunction_)(msgReader);

        check(zmq_msg_close(&msg), "zmq_msg_close");
    }
}

} // namespace capnzero
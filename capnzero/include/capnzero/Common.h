#pragma once

#include <zmq.h>

#include <iostream>
#include <string>

namespace capnzero
{
enum Protocol
{
    UDP,
    TCP,
    IPC
};

static const u_int8_t MAX_TOPIC_LENGTH = 16;

/**
 * Checks the return code and reports an error if present.
 * If abortIfError is set to true, it also aborts the process.
 */
inline void check(int returnCode, std::string methodName)
{
    if (returnCode != 0) {
        std::cerr << methodName << " returned: " << errno << " means: " << zmq_strerror(errno) << std::endl;
    }
}

/**
 * If there was an error sending the given message (numOfBytes == -1), the given message is closed, in order
 * to avoid memory leaks.
 * @param numBytesSent Number of bytes sent, according to zeromq return value.
 * @param msg The message that was sent.
 * @param sender String for debugging purposes.
 * @return Number of bytes actually sent. (0 in case of error)
 */
inline int checkSend(int numBytesSent, zmq_msg_t& msg, std::string sender)
{
    if (numBytesSent == -1) {
        std::cerr << sender << " zmq_msg_send was unsuccessful: " << errno << " means: " << zmq_strerror(errno) << std::endl;
        check(zmq_msg_close(&msg), "zmq_msg_close");
        return 0;
    } else {
        return numBytesSent;
    }
}

/**
 * If there was an error or timeout receiving the given message (numBytesReceived == -1), the given message is closed,
 * in order to avoid memory leaks.
 * @param numBytesReceived Number of bytes received, according to zeromq return value.
 * @param msg The message that was received.
 * @param receiver String for debugging purposes.
 * @return Number of bytes actually received. (0 in case of error or timeout)
 */
inline int checkReceive(int numBytesReceived, zmq_msg_t& msg, std::string receiver)
{
    if (numBytesReceived == -1) {
        if (errno != EAGAIN) // receiving a message was unsuccessful
        {
            std::cerr << receiver << " zmq_msg_recv received no bytes: " << errno << " means " << zmq_strerror(errno) << std::endl;
        }
#ifdef DEBUG_SUBSCRIBER
        else { // no message available
            std::cout << "Subscriber::receive(): continue because of EAGAIN!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << ".";
        std::cout.flush();
#endif
        check(zmq_msg_close(&msg), "zmq_msg_close");
        return 0;
    }
    else
    {
        return numBytesReceived;
    }
}
} // namespace capnzero

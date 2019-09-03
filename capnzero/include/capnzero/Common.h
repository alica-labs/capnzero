#pragma once

#include <zmq.h>

#include <iostream>
#include <string>

namespace capnzero
{
    enum Protocol {
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
 * If there was an error sending the given message (numberOfBytes == -1), the given message is closed, in order
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
} // namespace capnzero

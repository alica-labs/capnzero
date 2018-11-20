#include <assert.h>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <zmq.h>

void setup_test_environment(void)
{
#if defined _WIN32
#if defined _MSC_VER
    _set_abort_behavior(0, _WRITE_ABORT_MSG);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif
#else
#if defined ZMQ_HAVE_CYGWIN
    // abort test after 121 seconds
    alarm(121);
#else
#if !defined ZMQ_DISABLE_TEST_TIMEOUT
    // abort test after 60 seconds
//    alarm(60);
#endif
#endif
#endif
#if defined __MVS__
    // z/OS UNIX System Services: Ignore SIGPIPE during test runs, as a
    // workaround for no SO_NOGSIGPIPE socket option.
    signal(SIGPIPE, SIG_IGN);
#endif
}

int msg_send(zmq_msg_t* msg_, void* s_, const char* group_, const char* body_)
{
    int rc = zmq_msg_init_size(msg_, strlen(body_));
    if (rc != 0)
        return rc;

    memcpy(zmq_msg_data(msg_), body_, strlen(body_));

    rc = zmq_msg_set_group(msg_, group_);
    if (rc != 0) {
        zmq_msg_close(msg_);
        return rc;
    }

    rc = zmq_msg_send(msg_, s_, 0);

    zmq_msg_close(msg_);

    return rc;
}

int msg_recv_cmp(zmq_msg_t* msg_, void* s_, const char* group_, const char* body_)
{
    int rc = zmq_msg_init(msg_);
    if (rc != 0)
        return -1;

    int recv_rc = zmq_msg_recv(msg_, s_, 0);
    if (recv_rc == -1)
        return -1;

    if (strcmp(zmq_msg_group(msg_), group_) != 0) {
        zmq_msg_close(msg_);
        return -1;
    }

    char* body = (char*) malloc(sizeof(char) * (zmq_msg_size(msg_) + 1));
    memcpy(body, zmq_msg_data(msg_), zmq_msg_size(msg_));
    body[zmq_msg_size(msg_)] = '\0';

    if (strcmp(body, body_) != 0) {
        zmq_msg_close(msg_);
        return -1;
    }

    zmq_msg_close(msg_);
    free(body);
    return recv_rc;
}

int main(void)
{
    setup_test_environment();
    void* ctx = zmq_ctx_new();
    assert(ctx);

    void* radio = zmq_socket(ctx, ZMQ_RADIO);
    void* dish = zmq_socket(ctx, ZMQ_DISH);

    int rc = zmq_bind(radio, "tcp://127.0.0.1:5556");
    assert(rc == 0);

    //  Leaving a group which we didn't join
    rc = zmq_leave(dish, "Movies");
    assert(rc == -1);

    //  Joining too long group
    char too_long_group[ZMQ_GROUP_MAX_LENGTH + 2];
    for (int index = 0; index < ZMQ_GROUP_MAX_LENGTH + 2; index++)
        too_long_group[index] = 'A';
    too_long_group[ZMQ_GROUP_MAX_LENGTH + 1] = '\0';
    rc = zmq_join(dish, too_long_group);
    assert(rc == -1);

    // Joining
    rc = zmq_join(dish, "Movies");
    assert(rc == 0);

    // Duplicate Joining
    rc = zmq_join(dish, "Movies");
    assert(rc == -1);

    // Connecting
    rc = zmq_connect(dish, "tcp://127.0.0.1:5556");
    assert(rc == 0);

    // msleep (SETTLE_TIME);

    zmq_msg_t msg;

    //  This is not going to be sent as dish only subscribe to "Movies"
    rc = msg_send(&msg, radio, "TV", "Friends");
    assert(rc == 7);

    //  This is going to be sent to the dish
    rc = msg_send(&msg, radio, "Movies", "Godfather");
    assert(rc == 9);

    //  Check the correct message arrived
    rc = msg_recv_cmp(&msg, dish, "Movies", "Godfather");
    assert(rc == 9);

    //  Join group during connection optvallen
    rc = zmq_join(dish, "TV");
    assert(rc == 0);

    zmq_sleep(1);

    //  This should arrive now as we joined the group
    rc = msg_send(&msg, radio, "TV", "Friends");
    assert(rc == 7);

    //  Check the correct message arrived
    rc = msg_recv_cmp(&msg, dish, "TV", "Friends");
    assert(rc == 7);

    //  Leaving groupr
    rc = zmq_leave(dish, "TV");
    assert(rc == 0);

    zmq_sleep(1);

    //  This is not going to be sent as dish only subscribe to "Movies"
    rc = msg_send(&msg, radio, "TV", "Friends");
    assert(rc == 7);

    //  This is going to be sent to the dish
    rc = msg_send(&msg, radio, "Movies", "Godfather");
    assert(rc == 9);

    // test zmq_poll with dish
    zmq_pollitem_t items[] = {
            {radio, 0, ZMQ_POLLIN, 0}, // read publications
            {dish, 0, ZMQ_POLLIN, 0},  // read subscriptions
    };
    rc = zmq_poll(items, 2, 2000);
    assert(rc == 1);
    assert(items[1].revents == ZMQ_POLLIN);

    //  Check the correct message arrived
    rc = msg_recv_cmp(&msg, dish, "Movies", "Godfather");
    assert(rc == 9);

    rc = zmq_close(dish);
    assert(rc == 0);

    rc = zmq_close(radio);
    assert(rc == 0);

    rc = zmq_ctx_term(ctx);
    assert(rc == 0);
    std::cout << "Test success" << std::endl;

    return 0;
}

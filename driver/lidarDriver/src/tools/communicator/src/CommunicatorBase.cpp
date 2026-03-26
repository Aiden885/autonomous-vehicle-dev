#include "CommunicatorBase.h"
#include <string.h>
#include <iostream>

CommunicatorBase::CommunicatorBase()
{
    this->context = zmq_ctx_new();
}

CommunicatorBase::~CommunicatorBase()
{
}

void CommunicatorBase::InitPublisher(int port, const char *ip)
{
    pub_port = port;
    publisher = zmq_socket(this->context, ZMQ_PUB);
    string ip_port = string("tcp://") + ip + string(":") + to_string(port);
    zmq_bind(publisher, ip_port.c_str());
}

void CommunicatorBase::InitSubscriber(int port, const char *ip)
{
    sub_port = port;
    subscriber = zmq_socket(this->context, ZMQ_SUB);
    string ip_port = string("tcp://") + ip + string(":") + to_string(port);
    zmq_connect(subscriber, ip_port.c_str());
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
}

void CommunicatorBase::publish(string &str_to_send)
{
    if (pub_port > 0)
    {
        size_t size = str_to_send.size();
        zmq_msg_t msg;
        zmq_msg_init_size(&msg, size);
        memcpy(zmq_msg_data(&msg), str_to_send.c_str(), size);
        // std::cout << "size = " << size << std::endl;
        zmq_msg_send(&msg, publisher, 0);
        zmq_msg_close(&msg);
    }
    else
    {
        cerr << "you must call InitPublisher function first!" << endl;
    }
}

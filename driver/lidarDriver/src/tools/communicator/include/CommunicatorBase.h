#ifndef COMMUNICATORBASE_H
#define COMMUNICATORBASE_H

#include <zmq.h>
#include <string>

using namespace std;

class CommunicatorBase
{
private:
    void *context;

protected:
    void *publisher;
    void *subscriber;

    int pub_port;
    int sub_port;

public:
    CommunicatorBase();
    void InitPublisher(int port, const char *ip = "127.0.0.1");
    void InitSubscriber(int port, const char *ip = "127.0.0.1");

    void publish(string &str_to_send);
    virtual void subscribe() = 0;

    virtual ~CommunicatorBase();
};

#endif /*COMMUNICATORBASE_H*/
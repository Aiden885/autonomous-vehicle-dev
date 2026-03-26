#ifndef ZMQCOMMUNICATION_H
#define ZMQCOMMUNICATION_H
#include <QThread>
#include <QObject>

// IMU相对车身质心的位置，向右为X正，向前为Y正
#define offsetX 0.0
#define offsetY 0.0

class ZmqCommunication : public QThread
{
    Q_OBJECT
public:
    explicit ZmqCommunication();
    void run();
    void stop();
    //void gaussConvert(double longitude1, double latitude1, double &x1, double &y1);
    void offsetRevise(double yaw, double &x1, double &y1);
private:
    volatile bool stopped;
};



#endif

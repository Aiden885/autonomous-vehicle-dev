#ifndef QIANXUN_H
#define QIANXUN_H
#include <QThread>
#include <QObject>
#include <QDebug>
#include "globalvariable.h"
#include <QtNetwork/QTcpSocket>
#include <QtSerialPort/QSerialPort>

//class MainWindow;   // 前置声明


using namespace std;
class QianXun : public QObject
{
    Q_OBJECT
public:
    explicit QianXun(QObject *parent = 0);
    ~QianXun();
    std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
    int outnmea_gga(unsigned char *buff);
//    void run();
//    void stop();

    QTcpSocket * socket;
    bool regist = false;
private:
//    volatile bool stopped;
    //QTcpSocket * socket;

    unsigned char *recBuf;//TCP获取的byte数据放入recBuf中，达到一整包时再处理,这就是byte类型
    int QianXunCount = 0;
    std::string base64_decode(std::string const& encoded_string);

public:
    QSerialPort* m_serialPortDiff;//receire GGA and send diff data 差分数据接口
    //MainWindow * mwParent;

private slots:
    void QianXun_connected();
    void QianXun_readyread();



};

#endif // QIANXUN_H

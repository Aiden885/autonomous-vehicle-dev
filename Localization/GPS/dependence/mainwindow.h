#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPainter>
#include <QPaintEvent>
#include <QTime>
#include <QTimer>
#include <QWidget>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <vector>
#include "globalvariable.h"
#include <sys/time.h>
#include "qianxun.h"

#include "gpfpd.h"
#include "gtime.h"
#include "gpgga.h"
#include "zmqcommunication.h"

using namespace std;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    void paintEvent(QPaintEvent *event);
    ~MainWindow();
    void openPort();//打开串口


public slots:
    void receiveInfoPosi();
    void receiveInfoGGA();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_record_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort* m_serialPortPosi;//IMU数据接收串口

    QStringList m_portNameList;

    QByteArray PortData;
    QByteArray PortDataShow;
    QByteArray PortDataGGA;

    QFile file;
    QTime startTime;
    int odoNum = 0;
    GPS InsD;
    QianXun *RTK;

    GPFPD gpfpd;
    GTIME gtime;
    GPGGA gpgga;
    bool bReadyGpfpd;
    bool bReadyGtime;

    void SetGPSdata(); //put gpfpd and gtime data into GInsDGPS


    ZmqCommunication *ZMQ;


public:
    QSerialPort* m_serialPortDiff;//receire GGA and send diff data 差分数据接口
    bool bReadyGpgga;
};

#endif // MAINWINDOW_H

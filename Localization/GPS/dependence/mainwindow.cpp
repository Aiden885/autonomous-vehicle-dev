#include <iostream>
#include <unistd.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include "globalvariable.h"


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //设置一个画图的时钟
    QTimer *timer = new QTimer(this);
    timer->start(1000);
    //必须放在mainwindow下，才能响应paintEvent函数
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    setWindowTitle("XLRTK");
    usleep(200000);

    bReadyGpfpd = false;
    bReadyGtime = false;
    bReadyGpgga = false;

    //这里直接发送了一个按钮消息
    on_pushButton_clicked();

}

void MainWindow::paintEvent(QPaintEvent *event)
{
    this->ui->label_port->setText(PortDataShow.toHex()); //使用label显示参数;
    ui->label_latlon->setText("Lat:" + QString::number(GInsDGPS.latitude, 10, 8) + "  Lon:" + QString::number(GInsDGPS.longitude, 10, 8) + "  altitude:" + QString::number(GInsDGPS.altitude, 10, 2));
    ui->label_azi->setText("SatNum:" + QString::number(GInsDGPS.satelliteNumber) +
                           "  locationStatus:" + GInsDGPS.locationStatus // QString::number(int(GInsDGPS.locationStatus),16)
                           + "  yawStatus:" + QString::number(GInsDGPS.yawStatus) + "  calibrationStatus:" + QString::number(GInsDGPS.calibrationStatus, 16));
    ui->label_spe->setText("yaw:" + QString::number(GInsDGPS.yaw, 10, 2) +
                           +"  Roll:" + QString::number(GInsDGPS.roll, 10, 2) + "  Pitch:" + QString::number(GInsDGPS.pitch, 10, 2) + "  PortNum:" + QString::number(PortDataNum) + "  PortLength:" + QString::number(PortDataLength));
    ui->label_spe2->setText("velocityKM:" + QString::number(GInsDGPS.velocity * 3.6, 10, 2) +
                            "  vEastKM:" + QString::number(GInsDGPS.velocityEast * 3.6, 10, 2) + "  vNorthKM:" + QString::number(GInsDGPS.velocityNorth * 3.6, 10, 2) + "  vDown:" + QString::number(GInsDGPS.velocityUp * 3.6, 10, 2));
    ui->label_RTKdata->setText("QianXun:" + QianXunRegist + "  " + QianXunCommunicate);

    //在千寻接收数据的线程里面，设置了QianXunCommunicateDog = 0
    //如果长时间不能接收到千寻的数据，重新创建一个千寻类对象
    if (QianXunCommunicateDog > 60)
    {
        qDebug() << " reconnect qianxu communication ";
        if(RTK != NULL)
            delete RTK;

        RTK = new QianXun(this);
        RTK->m_serialPortDiff = m_serialPortDiff;
        QianXunCommunicateDog = 0;
    }
    QianXunCommunicateDog++;
    qDebug() << " QianXunCommunicateDog " <<QianXunCommunicateDog;
}
void MainWindow::openPort()
{
    // 1打开第一个串口
    if (m_serialPortPosi->isOpen()) //如果串口已经打开了 先给他关闭了
    {
        m_serialPortPosi->clear();
        m_serialPortPosi->close();
    }
    m_serialPortPosi->setPortName(INSDGPSParam.PortName); //当前选择的串口名字
    if (!m_serialPortPosi->open(QIODevice::ReadWrite))    //用ReadWrite 的模式尝试打开串口
    {
        qDebug() << INSDGPSParam.PortName << "open failed!";
        return;
    }
    qDebug()<<"串口打开成功!" << INSDGPSParam.PortName;
    m_serialPortPosi->setBaudRate(115200, QSerialPort::AllDirections); //设置波特率和读写方向
    m_serialPortPosi->setDataBits(QSerialPort::Data8);                 //数据位为8位
    m_serialPortPosi->setFlowControl(QSerialPort::NoFlowControl);      //无流控制
    m_serialPortPosi->setParity(QSerialPort::NoParity);                //无校验位
    m_serialPortPosi->setStopBits(QSerialPort::OneStop);               //一位停止位s
    connect(m_serialPortPosi, SIGNAL(readyRead()), this, SLOT(receiveInfoPosi()));

    // 2打开第二个串口
    if (m_serialPortDiff->isOpen()) //如果串口已经打开了 先给他关闭了
    {
        m_serialPortDiff->clear();
        m_serialPortDiff->close();
    }
    m_serialPortDiff->setPortName(INSDGPSParam.PortNameRTK); //当前选择的串口名字
    if (!m_serialPortDiff->open(QIODevice::ReadWrite))       //用ReadWrite 的模式尝试打开串口
    {
        qDebug() << INSDGPSParam.PortNameRTK << "open failed!";
        return;
    }
    qDebug()<<"串口打开成功!" << INSDGPSParam.PortNameRTK;
    m_serialPortDiff->setBaudRate(QSerialPort::Baud115200, QSerialPort::AllDirections); //设置波特率和读写方向
    m_serialPortDiff->setDataBits(QSerialPort::Data8);                                  //数据位为8位
    m_serialPortDiff->setFlowControl(QSerialPort::NoFlowControl);                       //无流控制
    m_serialPortDiff->setParity(QSerialPort::NoParity);                                 //无校验位
    m_serialPortDiff->setStopBits(QSerialPort::OneStop);                                //一位停止位
    connect(m_serialPortDiff, SIGNAL(readyRead()), this, SLOT(receiveInfoGGA()));
}

//接收GGA数据，只是作为一个发送差分数据到惯导的定时驱动
//增加记录文件，用于与惯导数据的对比
void MainWindow::receiveInfoGGA()
{
    QByteArray info = m_serialPortDiff->readAll();
    qDebug() << "recrive gga data:" << info.length() << info;
    if (saveflag)
    {

        //txtOutput << QTime::currentTime().toString("hh:mm::ss.zzz") << ",";
        txtOutput << info;
        txtOutput.flush();

    }


    if (info.size() > 1000)
        return;               //第一次数据量太大，就抛弃

    PortDataGGA.append(info); //将这次得到的byte累加到历次得到的byte之后

    if (PortDataGGA.size() < 20)
        return; //小于70个字符肯定没接收完

    if (PortDataGGA.size() > 2000) //这里假设了接收端并不是很快
    {
        PortDataGGA.clear();
        return;
    }

    //
    while (PortDataGGA.size() > 6) //至少包含消息头的数据
    {
        if (PortDataGGA.startsWith("$GPGGA")) //找到消息头
        {
            int nIndex0D = PortDataGGA.indexOf((char)0x0D);
            int nIndex0A = PortDataGGA.indexOf((char)0x0A);

            if (nIndex0D > 0 && nIndex0A > 0 && (nIndex0D + 1) == nIndex0A) //找到消息尾,可能有多条数据
            {
                //解析这一段数据
                bReadyGpgga = gpgga.parse(PortDataGGA.left(1 + nIndex0A));
                qDebug()<<PortDataGGA.left(1 + nIndex0A);

                if(bReadyGpgga )//
                {
                    QByteArray strSend = PortDataGGA.left(1 + nIndex0A);
                    if(RTK->socket != NULL && RTK->regist ==true) //send gga data to qianxun
                    {
                        if(-1 == RTK->socket->write(strSend))
                            RTK->regist = false;
                    }
                    
                    bReadyGpgga = false;
                }

                //将已解析数据从buff里面删除
                PortDataGGA.remove(0, 1 + nIndex0A);
            }
            else //没有找到消息尾
            {
                if (PortDataGGA.length() > 1000) //如果很长都找不到尾，清空。
                    PortDataGGA.clear();
                else
                    break;
            }
        }
        else //没有找到消息头
        {
            PortDataGGA.remove(0, 1); //删除第一个字符
        }

    } // while(PortDataGGA.size() > 6)


}
//接收和解析惯导$GPFPD,GTIMU数据
void MainWindow::receiveInfoPosi()
{
    QByteArray info = m_serialPortPosi->readAll();
    //qDebug() << "recrive imu data:" << info.length() << info;
    if (saveflag)
    {
        //txtOutput << QTime::currentTime().toString("hh:mm::ss.zzz") << ",";
        txtOutput << info;
        txtOutput.flush();
    }

    if (info.size() > 1000)
        return; //第一次数据量太大，就抛弃

    PortData.append(info); //将这次得到的byte累加到历次得到的byte之后

    if (PortData.size() < 32)
        return;                 //小于32个字符肯定没接收完，其实这里不需要做这个判断，应该是数据包解析的功能

    if (PortData.size() > 2000) //这里假设了接收端并不是很快
    {
        PortData.clear();
        return;
    }

    //
    while (PortData.size() > 6) //至少包含消息头的数据
    {
        if (PortData.startsWith("$GPFPD") || PortData.startsWith("$GTIMU")) //找到消息头
        {
            int nIndex0D = PortData.indexOf((char)0x0D);
            int nIndex0A = PortData.indexOf((char)0x0A);

            if (nIndex0D > 0 && nIndex0A > 0 && (nIndex0D + 1) == nIndex0A) //找到消息尾,可能有多条数据
            {
                //解析这一段数据
                //decodeReceiveInfo(PortData.left(1 + nIndex0A));

                if(PortData.startsWith("$GPFPD"))
                {
                    bReadyGpfpd = gpfpd.parse(PortData.left(1 + nIndex0A));
                    //qDebug()<<PortData;
                }
                else if(PortData.startsWith("$GTIMU"))
                {
                    //qDebug()<<PortData;
                    bReadyGtime = gtime.parse(PortData.left(1 + nIndex0A));
                }

                if(bReadyGpfpd && bReadyGtime && gpfpd.lutctime == gtime.lutctime)//
                {
                    SetGPSdata();
                    mutexListGInsDGPS.lock();
                    listGInsDGPS.append(GInsDGPS);
                    mutexListGInsDGPS.unlock();

                    bReadyGpfpd = false;
                    bReadyGtime = false;

                    ZmqSendSemphore.release();
                }

                //将已解析数据从buff里面删除
                PortData.remove(0, 1 + nIndex0A);
            }
            else //没有找到消息尾
            {
                if (PortData.length() > 1000) //如果很长都找不到尾，清空。
                    PortData.clear();
                else
                    break;
            }
        }
        else //没有找到消息头
        {
            PortData.remove(0, 1); //删除第一个字符
        }

    } // while(PortData.size() > 6)
}


MainWindow::~MainWindow()
{

    file.close();

    if(RTK != NULL)
    {
        delete RTK;
        RTK = NULL;
    }



    m_serialPortPosi->close();
    m_serialPortPosi = NULL;

    m_serialPortDiff->close();
    m_serialPortDiff->close();

    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    //开启zmq通信线程
    ZmqCommunication *ZMQ = new ZmqCommunication();
    ZMQ->start();


    GPFPD mygpfpd;
    QString strSource = "$GPFPD,2218,194068.100,90.291,-0.433,-1.023,39.99989628,116.33022512,33.661,-0.002,0.000,-0.001,1.507,14,13,04*46\r\n";
    mygpfpd.parse(strSource);

    //打开两个串口，一个用于链接千寻
    m_serialPortPosi = new QSerialPort();
    m_serialPortDiff = new QSerialPort();
    openPort();

    //delete RTK;
    RTK = new QianXun(this);
    RTK->m_serialPortDiff = m_serialPortDiff;
    QianXunCommunicateDog = 0;

    ui->pushButton->setEnabled(false);
}

void MainWindow::on_pushButton_record_clicked()
{
    if(saveflag == false )//no log
    {
       //open file
        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_String = current_date_time.toString("yyyyMMddhhmmss") + ".txt";
        file.setFileName("./data/" + current_String);//位于当前文件夹内
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            return;
        }

       txtOutput.setDevice(&file);
       saveflag = true;
       ui->pushButton_record->setText("停止");
    }
    else
    {
      //  close file
      file.close();

      saveflag = false;
      ui->pushButton_record->setText("记录");
    }


}

void MainWindow::SetGPSdata() //put gpfpd and gtime data into GInsDGPS
{
    GInsDGPS.longitude = gpfpd.dLongitude;//经度 度
    GInsDGPS.latitude = gpfpd.dLattitude;//纬度 度
    GInsDGPS.altitude = gpfpd.dAltitude;//高度 m
    GInsDGPS.yaw = gpfpd.dHeading ;//航向角 rad  (-Pi,+Pi] 正北0
    GInsDGPS.roll = gpfpd.dRoll; //翻滚角 rad
    GInsDGPS.pitch = gpfpd.dPitch;//俯仰角 rad
    GInsDGPS.velocityNorth = gpfpd.dVn; //北向车速 m/s
    GInsDGPS.velocityEast = gpfpd.dVe; //东向车速 m/s
    GInsDGPS.velocityUp = gpfpd.dVu;//垂直地面速度 m/s
    GInsDGPS.velocity = gpfpd.dVelocityEN; //车速 m/s
    GInsDGPS.locationStatus = gpfpd.strStatus;//定位模式  50 NARROW_INT//XLGPS里面没有
    //GInsDGPS.yawStatus;//定向模式//XLGPS里面没有
    //GInsDGPS. char      calibrationStatus;//bit0: 位置 bit1: 速度  bit2: 姿态 bit3: 航向角 //XLGPS里面没有
    GInsDGPS.satelliteNumber = gpfpd.nNSV2;//卫星数量
    GInsDGPS.utime = gpfpd.lutctime; //by syp 20220720 时间，与XLGPS的定义一致

///////////////////////////////////////////
    //double gaussX;// = 3;
    //double gaussY;// = 4;
    GInsDGPS.gpsValid = gpfpd.nGNSSStatus;// = 5;

//add for test
    GInsDGPS.ltime = gpfpd.lutctime;// = 9;
    GInsDGPS.gyroX = gtime.dGyroX;// = 13;
    GInsDGPS.gyroY = gtime.dGyroY;// = 14;
    GInsDGPS.gyroZ = gtime.dGyroZ;// = 15;
    GInsDGPS.accX = gtime.dAccX;// = 16;
    GInsDGPS.accY = gtime.dAccY;// = 17;
    GInsDGPS.accZ = gtime.dAccZ;// = 18;
}

#include "qianxun.h"
#include <unistd.h>
//#include "mainwindow.h"


QianXun::QianXun(QObject *parent)
{
    //parentMainWindow = (MainWindow *) parent;
    m_serialPortDiff = NULL;

    recBuf = new unsigned char[1000];
    this->socket = new QTcpSocket(this);
    //原来是8001端口，这个是不是设置错了，还是原来的坐标框架用的就是这个？
    // this->socket->connectToHost("203.107.45.154",8001,QTcpSocket::ReadWrite);
    //this->socket->connectToHost("203.107.45.154", 8002, QTcpSocket::ReadWrite);
    this->socket->connectToHost(INSDGPSParam.NTripCasterIP, INSDGPSParam.NTripCasterPort, QTcpSocket::ReadWrite);
    connect(this->socket, SIGNAL(connected()), this, SLOT(QianXun_connected()));
}
QianXun::~QianXun()
{
    socket->close();
    delete socket;
}
void QianXun::QianXun_connected()
{
    usleep(1000000);
    connect(this->socket, SIGNAL(readyRead()), this, SLOT(QianXun_readyread()));
    std::string str1 = "GET /RTCM32_GGB HTTP/1.1\r\n";
    std::string str2 = "User-Agent: NTRIP client.py/0.1\r\n";
    std::string str3 = "Authorization: Basic ";
    std::string usr_pwd = INSDGPSParam.usr_pwd.toStdString(); //"qxscpq001:d19180d";
    unsigned char *pp = (unsigned char *)usr_pwd.c_str();
    std::string user_bas64 = base64_encode(pp, strlen((const char *)pp));
    QString str = QString::fromStdString(str1 + str2 + str3 + user_bas64 + "\r\n\r\n");
    socket->write(str.toLatin1());
}
void QianXun::QianXun_readyread()
{
    //读取TCP　数据
    //    QMessageBox::about(this,"提示","准备读取");
    QByteArray da = this->socket->readAll();
    QString RecData = da;
    qDebug() <<  "qianxun recevied" <<RecData;

    QianXunCommunicateDog = 0;

    if (RecData.contains("ICY 200 OK"))
    {
        regist = true; //账号密码正确
        QianXunRegist = "regist";
        //usleep(2000000); //注册成功后修整1秒，用于等待串口数据到达

    }
    else if (RecData.contains("Unauthorized"))
    {
        regist = false; //账号密码不正确
        QianXunRegist = "password error";
    }
    else
    {
        //这里将数据差分数据打包，并不发送给接收机和惯导
        //而是在接收机和惯导收到串口数据的时候，将差分数据发送过去，哭了
        //应该将串口指针传递到这边，如果串口是打开的，直接将数据发送过去
        //    if(regist & GPGGAFlag)    {//当得到串口数据时，才发送给千寻
        if ( regist  && m_serialPortDiff != NULL)
        {
           //这里将千寻返回的差分数据发送给接收机
            //if (QianXunFlag)
            {
                int nWrite = m_serialPortDiff->write(da);
                //QianXunFlag = false;
            }

            QianXunData = da;
            //GPGGAFlag = false;
            //QianXunFlag = true;
            QianXunCommunicate = QString::number(QianXunCount);
            QianXunCount++;

            //qDebug()<<"send dato to qianxun" <<QianXunCommunicateDog;
        }
    }


}

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";
static inline bool is_base64(unsigned char c)
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}
std::string QianXun::base64_encode(unsigned char const *bytes_to_encode, unsigned int in_len)
{
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    while (in_len--)
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (i = 0; (i < 4); i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }
    if (i)
    {
        for (j = i; j < 3; j++)
            char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];
        while ((i++ < 3))
            ret += '=';
    }
    return ret;
}
std::string QianXun::base64_decode(std::string const &encoded_string)
{
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;
    while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_]))
    {
        char_array_4[i++] = encoded_string[in_];
        in_++;
        if (i == 4)
        {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
            for (i = 0; (i < 3); i++)
                ret += char_array_3[i];
            i = 0;
        }
    }
    if (i)
    {
        for (j = i; j < 4; j++)
            char_array_4[j] = 0;
        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);
        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
        for (j = 0; (j < i - 1); j++)
            ret += char_array_3[j];
    }
    return ret;
}

//在这里对GGA数据进行打包，以后应该单独封装为类
//由于数据传输采用的是全局变量，这里面也简单同样进行处理了
int QianXun::outnmea_gga(unsigned char *buff)
{
    if (buff == NULL)
        return 0;

    //当不定位的时候，这里不返回无内容的空信息，$GPGGA,,,,,,,,,,,,,,
    //直接返回0，后续也不用处理了
    //这部分用于验证时间的真确性
    if (GInsDGPS.utime == 0)
        return 0;

    //时间数据准备
    QDateTime gnssBeginTime(QDate(1980, 1, 6), QTime(0, 0, 0));
    QDateTime gnssTime = gnssBeginTime.addMSecs(GInsDGPS.utime);
    QDateTime utcTime = gnssTime.addSecs(-18); //-跳秒 -18

    //经纬度数据准备，转化为度分形式
    double ddmmLong, ddmmLati;
    ddmmLong = floor(GInsDGPS.longitude) * 100 +
               (GInsDGPS.longitude - floor(GInsDGPS.longitude)) * 60 * 100;
    ddmmLati = floor(GInsDGPS.latitude) * 100 +
               (GInsDGPS.latitude - floor(GInsDGPS.latitude)) * 60 * 100;

    char *p = (char *)buff;
    char *q, sum; //计算奇偶校验的临时变量 结果变量
    //$GPGGA,092949.00,4000.4046369,N,11619.6793071,E,5,11,1.0,49.387,M,-9.70,M,02,2821*72\r\n
    //                   时间 纬度度  分   N    经度度   分   E 状态卫星数DOP 高度 M  高程异常M 临期(无) 基准站ID（无）
    p += sprintf(p, "$GPGGA,%s,%02.0f%010.7f,N,%03.0f%010.7f,E,1,%02d,1.0,49.387,M,-9.70,M,,",
                 utcTime.time().toString("HHmmss.zzz").toLatin1().data(), //时分秒.毫秒
                 floor(GInsDGPS.latitude),                                //纬度度
                 (GInsDGPS.latitude - floor(GInsDGPS.latitude)) * 60,     //分
                 floor(GInsDGPS.longitude),                               //经度度
                 (GInsDGPS.longitude - floor(GInsDGPS.longitude)) * 60,   //分
                 GInsDGPS.satelliteNumber);                               //卫星数
    //计算校验位
    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
    return p - (char *)buff;
}

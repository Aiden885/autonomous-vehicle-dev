// for thread and timer:
#include "imu.pb.h"
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string.h>
// for zmq:
#include <zmq.h>
// for json:
#include <jsoncpp/json/json.h>

// for serial communication
#include <errno.h>
#include <fcntl.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// for save data

#include <fstream>
#include <iostream>

#define TIMER_PERIOD 100
#define SAVE_FILE 1

// GPS相对车身质心的位置，向右为X正，向前为Y�?
#define offsetX 0.0
#define offsetY 0.0

struct DGPS
{
    double utc_second;
    double latitude;
    double longitude;
    double heading;
    double X;
    double Y;
    int satNum;
    int status_main; // 主站
    int status_vice; // 从站
    double speed;    // add by syp
};

// add by shyp 20220830 formular from Fusion program
void gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y)
{

    double a = 6378137.0;

    double e2 = 0.0066943799013;

    double latitude2Rad = (M_PI / 180.0) * latitude1;

    int beltNo = int((longitude1 + 1.5) / 3.0);
    int L = beltNo * 3;
    double l0 = longitude1 - L;
    double tsin = sin(latitude2Rad);
    double tcos = cos(latitude2Rad);
    double t = tan(latitude2Rad);
    double m = (M_PI / 180.0) * l0 * tcos;
    double et2 = e2 * pow(tcos, 2);
    double et3 = e2 * pow(tsin, 2);
    double X = 111132.9558 * latitude1 - 16038.6496 * sin(2 * latitude2Rad) + 16.8607 * sin(4 * latitude2Rad) - 0.0220 * sin(6 * latitude2Rad);
    double N = a / sqrt(1 - et3);

    dNorth_X = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    dEast_Y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
    //   std::cout << BOLDRED << "x1: " << x1 << std::endl;
    //   std::cout << BOLDRED << "y1: " << y1 << std::endl;
}

int hex2dec(char ch)
{
    if ('0' <= ch && ch <= '9')
        return ch - '0';
    if ('A' <= ch && ch <= 'F')
        return ch - 'A' + 10;
    return -1;
}

void set_speed(int fd, int speed)
{
    int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B1200, B300, B38400, B19200, B9600, B4800, B1200, B300};
    int name_arr[] = {115200, 38400, 19200, 9600, 4800, 1200, 300, 38400, 19200, 9600, 4800, 1200, 300};
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (int i = 0; i < (sizeof(speed_arr) / sizeof(int)); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0)
            {
                perror("tcsetattr fd");
                std::cout << "set_speed err" << std::endl;
                return;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
}

int set_port(int fd, int databits, int stopbits, char parity)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return 0;
    }

    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 'S':
    case 's':
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return -1;
    }

    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return -1;
    }

    if (parity != 'n')
        options.c_iflag |= INPCK;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 141111: Binary mode to disable auto adding '0d' before '0a'
    options.c_oflag &= ~(INLCR | IGNCR | ICRNL);
    options.c_oflag &= ~(ONLCR | OCRNL);

    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("SetupSerial 3");
        return -1;
    }
    printf("The port is working at: %d, %c, %d.\n", databits, parity, stopbits);
    return 0;
}

// 校正高斯坐标中GPS相对于车辆质心的位置偏移，offset向右为X正，向前为Y�?
void offsetRevise(double yaw, double &x1, double &y1)
{
    double yaw2Rad = (M_PI / 180.0) * yaw;
    x1 -= offsetY * cos(yaw2Rad) - offsetX * sin(yaw2Rad);
    y1 -= offsetY * sin(yaw2Rad) + offsetX * cos(yaw2Rad);
}

class Jobs
{
  public:
    explicit Jobs(boost::asio::io_context &io, int f, std::vector<void *> &sendingSocketList) : strand(io), timer(io, boost::posix_time::milliseconds(5)), fd(f), sSocketList(sendingSocketList)
    {
        printf("error saveFileStream.open--/datafile.txt\n");
        // open file
        if (SAVE_FILE)
        {
            saveFileStream.open("./datafile.txt");
            printf("saveFileStream.open--/datafile.txt\n");
        }
        else
        {
            printf("error saveFileStream.open--/datafile.txt\n");
        }

        timer.async_wait(strand.wrap(boost::bind(&Jobs::serialSubscriber, this)));
    }

    ~Jobs()
    {
        if (SAVE_FILE && saveFileStream.is_open())
        {
            saveFileStream.close();
        }
    }

#define MSGSIZE 300
    void serialSubscriber()
    {
        boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
        // read a valid message
        char msg[MSGSIZE] = "";

        while (true)
        {
            int msglen = read(fd, msg, MSGSIZE);

            if (msg[0] == '$')
            {
                buf.clear();

                if (msglen > 5 && (msg[msglen - 5] == '*') && (msg[msglen - 2] == '\n') && (msg[msglen - 1] == '\n'))
                {
                    if (!Parser(msg))
                        break;
                }
                else
                {
                    std::string msg_str = msg;
                    std::size_t pos = msg_str.find('$', 1);
                    if (pos != std::string::npos)
                    {
                        buf += msg_str.substr(pos, msg_str.size() - pos);
                        if (!Parser(msg_str.substr(0, pos).c_str()))
                            break;
                    }
                    else
                    {
                        buf += msg_str;
                    }
                }
            }
            else
            {
                buf += std::string(msg);
                size_t pos_st = buf.find('$');
                size_t pos_end = buf.find('\n');
                if ((pos_st != std::string::npos) && (pos_end != std::string::npos))
                {
                    // std::cout << "5555" << std::endl;
                    std::cout << buf << std::endl;

                    // char msg_tmp[MSGSIZE] = "";
                    // strncpy(msg_tmp, buf.c_str(), pos_end + 2 - pos_st);
                    // buf.erase(pos_st, pos_end + 2 - pos_st);

                    if (!Parser(buf.substr(pos_st, pos_end + 2 - pos_st).c_str()))
                    {
                        buf.erase(pos_st, pos_end + 2 - pos_st);
                        break;
                    }
                    buf.erase(pos_st, pos_end + 2 - pos_st);
                }
            }

            break;
        }

        boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration duration = end - start;
        int16_t duration_t = duration.total_milliseconds();
        duration_t >= TIMER_PERIOD ? TIMER_PERIOD : duration_t;
        timer.expires_from_now(boost::posix_time::milliseconds(TIMER_PERIOD - duration_t));
        timer.async_wait(strand.wrap(boost::bind(&Jobs::serialSubscriber, this)));
    }

    bool Parser(const char *msg_)
    {
        int msglen, tmp, comma[25], commaNum;
        char header[20] = "";
        uint8_t checksum, nowsum;
        char *ptr = nullptr;

        msglen = strlen(msg_);

        tmp = hex2dec(msg_[msglen - 4]);
        if (tmp == -1)
            return false;
        checksum = tmp;

        tmp = hex2dec(msg_[msglen - 3]);
        if (tmp == -1)
            return false;
        checksum = checksum * 16 + tmp;

        nowsum = 0;
        for (int i = 1; i < msglen - 5; i++)
            nowsum ^= msg_[i];

        if (nowsum != checksum)
        {
            std::cout << "Checksum Failed! nowsum = " << nowsum << ", checksum = " << checksum << std::endl;
            return false;
        }

        commaNum = 0;
        for (int i = 6; i < msglen - 5; i++) // get positions of all commas
        {
            if (msg_[i] == ',')
                comma[commaNum++] = i;
        }

        memset(header, 0, sizeof(header));
        strncpy(header, msg_ + 1, 5);

        if (strcmp(header, "GPYBM") == 0)
        {
            if (commaNum != 23)
                return false;

            // UTC
            double dtmp = strtod(msg_ + comma[1] + 1, &ptr);
            gps.utc_second = ((int16_t)(dtmp / 10000) + 8) * 3600 + ((int16_t)(dtmp / 100)) % 100 * 60 + fmod(dtmp, 100);
            std::cout << "utc_second = " << std::fixed << std::setprecision(3) << gps.utc_second << std::endl;

            // latitude
            gps.latitude = strtod(msg_ + comma[2] + 2, &ptr); // 去掉'+'�??
            // std::cout << "latitude = " << std::fixed << std::setprecision(9) << gps.latitude
            //           << std::endl;

            // longitude
            gps.longitude = strtod(msg_ + comma[3] + 2, &ptr); // 去掉'+'�??
            // std::cout << "longitude = " << std::fixed << std::setprecision(9) << gps.longitude
            //           << std::endl;

            // YAW
            gps.heading = strtod(msg_ + comma[5] + 1, &ptr);
            // std::cout << "Yaw = " << std::fixed << std::setprecision(3) << gps.heading <<
            // std::endl;

            // X velocity
            // dtmp = strtod(msg_ + comma[7] + 1, &ptr);
            // std::cout << "X velocity = " << std::fixed << std::setprecision(3) << dtmp <<
            // std::endl;

            // Y velocity
            // dtmp = strtod(msg_ + comma[8] + 1, &ptr);
            // std::cout << "Y velocity = " << std::fixed << std::setprecision(3) << dtmp <<
            // std::endl;

            // add by spy
            //  speed
            gps.speed = strtod(msg_ + comma[10] + 1, &ptr);
            std::cout << "speed  = " << std::fixed << std::setprecision(3) << gps.speed << std::endl;

            // Gauss X
            gps.X = strtod(msg_ + comma[11] + 1, &ptr);
            std::cout << "X = " << std::fixed << std::setprecision(3) << gps.X << std::endl;

            // Gauss Y
            gps.Y = strtod(msg_ + comma[12] + 1, &ptr);
            std::cout << "Y = " << std::fixed << std::setprecision(3) << gps.Y << std::endl;

            // status
            char ch = msg_[comma[15] + 1];
            if (ch < '0' || '5' < ch)
                return false;
            gps.status_main = ch - '0';
            // std::cout << "status_main = " << gps.status_main << std::endl;

            ch = msg_[comma[16] + 1];
            if (ch < '0' || '5' < ch)
                return false;
            gps.status_vice = ch - '0';
            // std::cout << "status_vice = " << gps.status_vice << std::endl;

            // satNum
            gps.satNum = strtod(msg_ + comma[17] + 1, &ptr);
            // std::cout << "satnum = " << gps.satNum << std::endl;

            Publisher();
        }

        return true;
    }

    void Publisher()
    {
        // send data by Json
        //  Json::Value values;

        // values["Gps"]["Latitude"] = gps.latitude;
        // values["Gps"]["Longitude"] = gps.longitude;
        // values["Gps"]["X"] = gps.X;
        // values["Gps"]["Y"] = gps.Y;
        // values["Gps"]["StatusMain"] = gps.status_main;
        // values["Gps"]["StatusVice"] = gps.status_vice;
        // values["Gps"]["Heading"] = gps.heading;
        // values["Gps"]["IsValid"] = true;
        // values["Gps"]["UtcSecond"] = gps.utc_second;

        // Json::FastWriter fastWriter;
        // std::string jsonString = fastWriter.write(values);
        // zmq_send(sSocketList[0], jsonString.c_str(), strlen(jsonString.c_str()), 0);
        // printf("UTC=%i:%i:%4.1lf  ", (int16_t)(gps.utc_second / 3600), (int16_t)(fmod(gps.utc_second, 3600) / 60), fmod(gps.utc_second, 60));
        // printf("lat=%.8lf lon=%.8lf hdg=%.2lf status=%d\n", gps.latitude, gps.longitude, gps.heading, gps.status_main);

        // send data by protobuf
        IMU::Imu imu;

        // double gaussX, gaussY;
        // gaussConvert(gps.longitude, gps.latitude, gaussX, gaussY);
        gaussConvert(gps.longitude, gps.latitude, gps.X, gps.Y);

        offsetRevise(gps.heading, gps.X, gps.Y);

        imu.set_longitude(gps.longitude);
        imu.set_latitude(gps.latitude);
        imu.set_gaussx(gps.X);
        imu.set_gaussy(gps.Y);

        if (gps.status_main == 4 && gps.status_vice == 4)
        {
            imu.set_gpsvalid(4); // both fixed
        }
        else if ((gps.status_main == 4 && gps.status_vice == 5) || (gps.status_main == 5 && gps.status_vice == 4))
        {
            imu.set_gpsvalid(5); // one fixed and one float
        }
        else
        {
            imu.set_gpsvalid(std::min(gps.status_main, gps.status_vice));
        }

        imu.set_time(gps.utc_second);
        imu.set_velocity(gps.speed);
        imu.set_yaw(gps.heading);

        size_t size = imu.ByteSize();
        void *buffer = malloc(size);

        // serialize your data, from pointVec to buffer
        if (!imu.SerializeToArray(buffer, size))
        {
            std::cerr << "Failed to write msg." << std::endl;
            return;
        }

        zmq_msg_t msg;
        zmq_msg_init_size(&msg, size);
        memcpy(zmq_msg_data(&msg), buffer, size); // copy data from buffer to zmq msg

        zmq_send(sSocketList[0], buffer, size, 0);
        printf("UTC=%i:%i:%4.1lf  ", (int16_t)(gps.utc_second / 3600), (int16_t)(fmod(gps.utc_second, 3600) / 60), fmod(gps.utc_second, 60));
        printf("lat=%.8lf lon=%.8lf hdg=%.2lf status=%d\n", gps.latitude, gps.longitude, gps.heading, gps.status_main);
        free(buffer);

        if (SAVE_FILE && saveFileStream.is_open())
        {

            saveFileStream << std::setprecision(15) << gps.longitude << " " << gps.latitude << " " << gps.X << " " << gps.Y << " " << gps.heading << " " << gps.status_main << " " << gps.status_vice
                           << " " << std::endl;
        }
    }

  private:
    boost::asio::io_context::strand strand;

    boost::asio::deadline_timer timer;

    std::vector<void *> sSocketList;

    int fd;
    DGPS gps;
    std::string buf;

    // add by shyp 20220830
    std::ofstream saveFileStream;
};

int main(int argc, char **argv)
{
    int fd = open("/dev/ttyS0", O_RDWR);
    if (-1 == fd)
    {
        printf("Error opening the port!\n");
        return -1;
    }
    else
    {
        printf("Serial port opened\n");
    }

    set_speed(fd, 115200);
    if (-1 == set_port(fd, 8, 1, 'N'))
    {
        printf("Set port error.\n");
    }
    tcflush(fd, TCIOFLUSH);

    // zmq communication
    void *context = zmq_ctx_new();

    // zmq sending sockets
    void *broadcastingSocket = zmq_socket(context, ZMQ_PUB);
    zmq_bind(broadcastingSocket, "tcp://127.0.0.1:5003");
    std::vector<void *> sendingSocketList;
    sendingSocketList.push_back(broadcastingSocket);

    // thread initial
    boost::asio::io_context io;
    Jobs jobs(io, fd, sendingSocketList);
    io.run();

    return 0;
}

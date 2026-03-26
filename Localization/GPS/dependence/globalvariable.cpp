#include "globalvariable.h"


GPS GInsDGPS;
QByteArray QianXunData;
bool QianXunFlag = false;
QString QianXunRegist = "disconnect";
QString QianXunCommunicate = "null";
int QianXunCommunicateDog = 0;
QByteArray GPGGAData;
//bool GPGGAFlag = true;
int PortDataNum = 0;
int PortDataLength = 0;
INSDGPSConfigureStruct INSDGPSParam;
QTime startTime;

//by syp 20220720
QList<GPS> listGInsDGPS;//用于解析数据与发送数据线程间数据传递
QMutex mutexListGInsDGPS;//用于解析数据与发送数据线程间数据传递的锁
QTextStream txtOutput;//用于记录文件的流
bool saveflag = false; //按下记录按钮
int savecount = 0;//惯导数据记录次数

QSemaphore ZmqSendSemphore;

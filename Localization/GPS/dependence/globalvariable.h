#ifndef GLOBALVARIABLE_H
#define GLOBALVARIABLE_H
#include <QWidget>
#include "datastruct.h"
#include "math.h"
#include <QTime>
#include <QMutex>
#include <QTextStream>
#include <QSemaphore>

extern GPS GInsDGPS;
extern QByteArray QianXunData;
extern bool QianXunFlag;
extern QString QianXunRegist;
extern QString QianXunCommunicate;
extern int QianXunCommunicateDog;
extern QByteArray GPGGAData;
extern bool GPGGAFlag;
extern int PortDataNum;
extern int PortDataLength;
extern QTime startTime;
extern INSDGPSConfigureStruct INSDGPSParam;
PositionXY CalculatePosition();

//by syp 20220720
extern QList<GPS> listGInsDGPS;//用于解析数据与发送数据线程间数据传递
extern QMutex mutexListGInsDGPS;//用于解析数据与发送数据线程间数据传递的锁
extern QTextStream txtOutput;//用于记录文件的流
extern bool saveflag; //按下记录按钮
extern int savecount;//惯导数据记录次数
extern QSemaphore ZmqSendSemphore;


#endif // GLOBALVARIABLE_H

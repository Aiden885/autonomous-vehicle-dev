#-------------------------------------------------
#
# Project created by QtCreator 2020-04-26T20:41:42
#
#-------------------------------------------------
#用ROS加这三句话
#INCLUDEPATH += /opt/ros/kinetic/include
#DEPENDPATH += /opt/ros/kinetic/include
#LIBS += -L/opt/ros/kinetic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime

#LIBS += -lpcap #使用pcap网络抓包必须添加此路径，且安装（sudo apt-get install libpcap-dev）,https://blog.csdn.net/tennysonsky/article/details/44811899
#LIBS += /usr/local/lib/libpcap.a    #我添加的！

#LIBS += -L /usr/lib-lprotobuf   #使用Protobuf时需要添加这两句，一个路径，一个源文件
#LIBS += -L /usr/local/lib/libprotobuf.a  #https://www.cnblogs.com/SmallBlackEgg/p/11628932.html

#LIBS += /usr/local/protobuf/lib/libprotobuf.a
#Xavier开启ROS使用如下三句话
#INCLUDEPATH += /opt/ros/melodic/include
#DEPENDPATH += /opt/ros/melodic/include
#LIBS += -L/opt/ros/melodic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime

QT       += core gui
QT += serialport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = INSDGPS
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

INCLUDEPATH += ../../../proto/Imu \

SOURCES += \
    ../../../proto/Imu/imu.pb.cc \
    src/globalvariable.cpp \
    src/gpfpd.cpp \
    src/gpgga.cpp \
    src/gtime.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/nmeadata.cpp \
    src/qianxun.cpp \
    src/configure.cpp \
    src/zmqcommunication.cpp \
    src/ztgeographycoordinatetransform.cpp

HEADERS += \
    ../../../proto/Imu/imu.pb.h \
    src/globalvariable.h \
    src/gpfpd.h \
    src/gpgga.h \
    src/gtime.h \
    src/mainwindow.h \
    src/datastruct.h \
    src/nmeadata.h \
    src/qianxun.h \
    src/configure.h \
    src/zmqcommunication.h \
    src/ztgeographycoordinatetransform.h

FORMS += \
    src/mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
#LIBS += -llcm
QT      +=network
LIBS += -lzmq
#LIBS += -ljsoncpp

INCLUDEPATH += /usr/local/include/google/protobuf
LIBS += -L /usr/local/lib -lprotobuf




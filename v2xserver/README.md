>

# V2xServer

高铁新城，获取路侧程序

## Description
包含如下内容：
SSMCallback：获取路侧感知物回调函数
SPATCallback:获取红绿灯数据回调函数
getPosFromVehicle:用于获取车辆位置信息
getGlobalPlanningFromVehicle:用于获取车辆发送的全局路径信息
publishSSM:用于读取路侧感知数据，并采用zmq的pub-sub模式，作为pub端发布给出
publishLight：用于读取路侧红绿灯数据，并采用zmq的pub-sub模式，作为pub端发布给出
## start and stop sh
脚本路径：/V2xServer/bin/
./start_v2x.sh 用于启动程序
./stop_v2x.sh  用于停止程序
## file config
/conf/setting.conf
ssmPubAddress=tcp://XXX:5581 #v2x发布感知物数据地址和端口 
spartPubAddress=tcp://xxx:5583 #v2x发布红绿灯数据地址和端口 
posSubAddress=tcp://xxx:5003 #车辆订阅感知物地址和端口
planningSubAddress=tcp://xxx:5582 #车辆订阅红绿灯地址和端口
serverReqAddress=tcp://xxx:5514 #路侧感知物数据上报云服务器地址和端口

## License
V1.0
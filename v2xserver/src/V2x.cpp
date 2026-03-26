#include "V2x.h"

std::map<int, Object> hisMap;
std::map<int, std::chrono::seconds> timeMap;
typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> miliClock_type;
V2x::V2x()
{
}
V2x::~V2x()
{
}
// verify successful login
bool V2x::init()
{
    if (Ivics::login() != 0)
    {
        std::cerr << "\033[1;31m????????????【Error】  Login to roadside failed,The program is exit!!]\033[0m" << std::endl;
        return false;
    }
    std::cout << "\033[1;32m "
              << "####【success】:  Login in roadside!!\033[0m " << std::endl;
    std::string str = Ivics::version();
    return true;
}

// Perception callback interface
void V2x::SSMCallback(const Ivics::SSM &sSM, ParameSct &ps)
{

    try
    {
        std::cout << "\033[1;34m "
                  << "*****【info】:  开始给车发送感知数据!!\033[0m " << std::endl;

        const Ivics::SSM msg_SSM = sSM; // Roadside information conversion
        RoadsideMsgVec roadsideMsg;
        SSM2Probuf(sSM, &roadsideMsg, ps); // Convert roadside information into proto
        // publish data to vechicle and write data to file
        if (roadsideMsg.rvecmsg_size() > 0)
        {

            std::cout << "\033[1;34m "
                      << "*****【info】:  过滤自车后感知物数量::" << roadsideMsg.rvecmsg_size() << "\033[0m " << std::endl;
            // 1 Publish data to the vehicle
            bool rs = publishSSM(&roadsideMsg, ps.pupbssm);
            if (rs)
            {
                //   std::cout << "\033[1;32m "
                //                           << "####【success】:  publish ssm data!!\033[0m " << std::endl;
            }
            else
            {
                std::cerr << "\033[1;31m????????????【Error】:   Failed publish  ssm data!!\033[0m" << std::endl;
            }
            // 2 Write data to log file
            if (ps.ssmWriteFlag == 1)
            {

                bool res = writeSSm2File(&roadsideMsg, ps.ssmFilePath);
                if (res)
                {
                    // std::cout << "\033[1;32m "
                    //                       << "####【success】:  Write ssm data to log file!!\033[0m " << std::endl;
                }
                else
                {
                    std::cerr << "\033[1;31m????????????【Error】:   Failed Write ssm data to log file!!\033[0m" << std::endl;
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m????????????【Error】:   SSMCallback   An error occurred::" << e.what() << "\033[0m" << std::endl;
    }
}
// Publish perception data to the vehicle
bool V2x::publishSSM(RoadsideMsgVec *msgvec, void *publisher)
{

    size_t size = msgvec->ByteSize();
    void *buffer = malloc(size);
    if (!msgvec->SerializeToArray(buffer, size))
    {
        std::cerr << "\033[1;31m????????????【Error】:   Failed publish SSM  to serializeToArray msg!!\033[0m" << std::endl;
        return false;
    }
    zmq_msg_t req;
    if (0 != zmq_msg_init_size(&req, size))
    {
        std::cerr << "\033[1;31m????????????【Error】:   Failed publish SSM zmq_msg_init !!\033[0m" << std::endl;
        return false;
    }
    memcpy(zmq_msg_data(&req), buffer, size);
    int len = zmq_msg_send(&req, publisher, 0);
    zmq_msg_close(&req);
    free(buffer);
    return true;
}

// Write perception data to log file
bool V2x::writeSSm2File(RoadsideMsgVec *msgvec, std::string filePath)
{
    std::ofstream partFile;
    partFile.open(filePath, std::ios::out | std::ios::app);
    try
    {
        partFile << "type";
        partFile << ",";
        partFile << "objectID";
        partFile << ",";
        partFile << "lat";
        partFile << ",";
        partFile << "lon";
        partFile << ",";
        partFile << "x";
        partFile << ",";
        partFile << "y";
        partFile << ",";
        partFile << "velocity";
        partFile << ",";
        partFile << "yaw";
        partFile << ",";
        partFile << "power";
        partFile << ",";
        partFile << "Finish";
        partFile << ",";
        partFile << "Len:";
        partFile << ",";
        partFile << "Width";
        partFile << ",";
        partFile << "Height";
        partFile << ",";
        partFile << "Timestamp";
        partFile << "\n";
        for (int i = 0; i < msgvec->rvecmsg_size(); i++)
        {
            const RoadsideMsg &fmsg = msgvec->rvecmsg(i);
            int Type = fmsg.type();
            int ObjectID = fmsg.objectid();
            double Lat = fmsg.lat();
            double Lon = fmsg.lon();
            double X = fmsg.x();
            double Y = fmsg.y();
            double Yaw = fmsg.yaw();
            double Velocity = fmsg.velocity();
            double Power = fmsg.power();
            bool Finish = fmsg.finish();
            int Len = fmsg.len();
            int Width = fmsg.width();
            int Height = fmsg.height();
            long long Timestamp = fmsg.timestamp();
            partFile << Type << ",";
            partFile << ObjectID << ",";
            partFile << std::setprecision(10) << Lat << ",";
            partFile << std::setprecision(10) << Lon << ",";
            partFile << std::setprecision(10) << X << ",";
            partFile << std::setprecision(10) << Y << ",";
            partFile << Velocity << ",";
            partFile << Yaw << ",";
            partFile << Power << ",";
            partFile << Finish << ",";
            partFile << Len << ",";
            partFile << Width << ",";
            partFile << Height << ",";
            partFile << Timestamp << "\n";
        }
        partFile.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m????????????【Error】:   writeSSm2File happen error::" << e.what() << "\033[0m" << std::endl;
        return false;
    }

    return true;
}

// write global planning data to file

bool V2x::writeGlobalPlanning2File(Planning::RoutingPointVec *routePointVec, std::string filePath)
{
    std::ofstream globlFile;
    globlFile.open(filePath, std::ios::out | std::ios::app);
    globlFile << "**********************************************" << std::endl;
    int rsize = routePointVec->routingpoints_size();
    globlFile << "*****size::" << rsize << std::endl;
    for (size_t i = 0; i < rsize; i++)
    {
        const ::Planning::RoutingPoint &routingPoint = routePointVec->routingpoints(i);
        double lat = routingPoint.latitude();
        double lon = routingPoint.longtitude();
        globlFile << std::setprecision(10) << "lon:" << lon << ",lat:" << lat << std::endl;
    }
    globlFile.close();
}

// Traffic light data subscription interface

void V2x::SPATCallback(const Ivics::SPAT &sPAT, ParameSct &ps)
{
    try
    {

        const Ivics::SPAT msg_SPAT = sPAT;
        infopack::SPAT spat_proto;
        // 1 trasfor data to protobuf
        Light2Probuf(sPAT, &spat_proto);

        if (spat_proto.ByteSizeLong() > 0)
        {
            static unsigned int publishLightCount = 0;

            // 2 publish data to vehicle
            bool rs = true;
            if ((publishLightCount++) % 3 == 0)
            {
                int inteLen = spat_proto.intersections_size();
                std::cout << "\033[1;34m "
                          << "*****【info】:  开始给车发送红绿灯数据!!\033[0m " << std::endl;
                std::cout << "\033[1;34m "
                          << "*****【info】:  获取红绿灯数量::" << inteLen << "\033[0m " << std::endl;
                rs = publishLight(&spat_proto, ps.pubspart);
            }
            if (rs)
            {
                //   std::cout << "\033[1;32m "
                //                           << "####【success】: publish light data!!\033[0m " << std::endl;
            }
            else
            {
                std::cerr << "?????????????Failed publish light data !" << std::endl;
            }
            // 3 write spat data to file
            if (ps.spartWriteFlag == 1)
            {
                bool res = writeLight2File(&spat_proto, ps.spartFilePath);
                if (res)
                {
                    // std::cout << " success Write spat light data to log file  !" << std::endl;
                }
                else
                {
                    std::cerr << "?????????????Failed Write spat light data to log file  !" << std::endl;
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m????????????【Error】:   SPATCallback  An error occurred:::" << e.what() << "\033[0m" << std::endl;
    }
}

bool V2x::publishLight(infopack::SPAT *spart, void *publisher)
{

    // 输出protobuff信息
    {
        size_t msgcnt = spart->msgcnt();
        size_t moy = spart->moy();
        size_t timeStamp = spart->timestamp();
        std::string name = spart->name();
        // std::cout << "*******msgcnt:" << msgcnt << ",moy:" << moy << ",timeStamp:" << timeStamp << ",name:" << name << std::endl;
        //  for (int i = 0; i < spart->intersections_size(); i++)
        //  {
        //      const infopack::IntersectionState intersectionState = spart->intersections(i);
        //      // std::cout << "spart.intersections(i).intersectionid " << intersectionState.intersectionid() << std::endl;
        //      for (int j = 0; j < spart->intersections(i).phases_size(); j++)
        //      {
        //          const infopack::Phase phase = spart->intersections(i).phases(j);
        //          std::cout << "phaseid " << phase.phaseid();
        //          std::cout << ", " << phase.lightstate();
        //          std::cout << ", " << phase.starttime();
        //          std::cout << ", " << phase.likelyendtime() << std::endl;
        //      }
        //  }
    }
    size_t size = spart->ByteSizeLong();
    void *buffer = malloc(size);
    if (!spart->SerializeToArray(buffer, size))
    {
        std::cerr << "\033[1;31m????????????【Error】:   Failed publish light to serializeToArray msg !\033[0m" << std::endl;
        return false;
    }
    zmq_msg_t req;
    if (0 != zmq_msg_init_size(&req, size))
    {
        std::cerr << "\033[1;31m????????????【Error】:   Failed publish light zmq_msg_init !\033[0m" << std::endl;
        free(buffer);
        return false;
    }
    memcpy(zmq_msg_data(&req), buffer, size);
    int len = zmq_msg_send(&req, publisher, 0);
    zmq_msg_close(&req);
    free(buffer);
    return true;
}

void V2x::Light2Probuf(const Ivics::SPAT &msg_SPAT, infopack::SPAT *spart)
{
    // get light data
    size_t cnt = msg_SPAT.msgCnt;
    int count = msg_SPAT.intersections.list.size();
    std::string name = msg_SPAT.name.c_str();
    size_t moy = 0; // minute of the year
    if (msg_SPAT.moy.get() != nullptr)
    {
        moy = *msg_SPAT.moy;
    }
    size_t timeStamp = 0; /// Time stamp when this message is formed in milliseconds
    if (msg_SPAT.timeStamp.get() != nullptr)
    {
        timeStamp = *msg_SPAT.timeStamp;
    }
    spart->set_msgcnt(cnt);
    spart->set_moy(moy);
    spart->set_timestamp(timeStamp);
    spart->set_name(name);
    for (int i = 0; i < count; i++)
    {
        std::string intersectionId = msg_SPAT.intersections.list[i].intersectionId; // road id
        infopack::IntersectionState *intersections;
        intersections = spart->add_intersections();
        intersections->set_intersectionid(intersectionId); // proto set intersectionId
        int phaseCount = msg_SPAT.intersections.list[i].phases.list.size();
        for (int j = 0; j < phaseCount; j++) // traverse  phase elements
        {
            size_t phases_id = msg_SPAT.intersections.list[i].phases.list[j].id;                          // get phase id
            int phaseStatesCount = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list.size(); // get light status nums
            infopack::Phase *phase;
            phase = intersections->add_phases();
            phase->set_phaseid(phases_id);
            for (int k = 0; k < phaseStatesCount; k++)
            {
                Ivics::LightState lightState = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].light; // 信号灯状态
                size_t startTime = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->startTime;
                size_t likelyEndTime = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->likelyEndTime;
                phase->set_lightstate((infopack::Phase_LightState)lightState);
                phase->set_starttime(startTime);
                phase->set_likelyendtime(likelyEndTime);
            }
        }
    }
}

bool V2x::writeLight2File(infopack::SPAT *spart, std::string filePath)
{
    std::ofstream lightFile;
    lightFile.open(filePath, std::ios::out | std::ios::app);
    try
    {
        lightFile << "***************开始输入信号灯信息****************" << std::endl;
        lightFile << "msgcnt:" << spart->msgcnt() << ",";
        lightFile << "moy:" << spart->moy() << ",";
        lightFile << "timeStamp:" << spart->timestamp() << ",";
        lightFile << "name:" << spart->name() << "\n";
        int inteLen = spart->intersections_size();
        for (int i = 0; i < inteLen; i++)
        {
            const infopack::IntersectionState &intersections = spart->intersections(i);
            lightFile << "roadId:" << intersections.intersectionid() << "\t";
            int phaseSize = intersections.phases_size();
            for (int j = 0; j < phaseSize; j++)
            {
                const infopack::Phase &phaseList = intersections.phases(j);
                lightFile << "相位ID:" << phaseList.phaseid() << "\t";
                lightFile << "红绿灯状态:" << phaseList.lightstate() << "\t";
                lightFile << "startTime:" << phaseList.starttime() << "\t";
                lightFile << "likelyendtime:" << phaseList.likelyendtime() << "\n";
            }
        }
        lightFile.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m????????????【Error】:   Failed write light data to file happy excetion::" << e.what() << "\033[0m" << std::endl;
        return false;
    }

    return true;
}

// void V2x::SPATCallback(const Ivics::SPAT &sPAT, ParameSct &ps)
// {

//     std::ofstream lightFile;

//     lightFile.open(ps.spartFilePath, std::ios::out | std::ios::app);
//     const Ivics::SPAT msg_SPAT = sPAT;
//     size_t cnt = msg_SPAT.msgCnt;
//     int count = msg_SPAT.intersections.list.size();
//     lightFile << "***************开始输入信号灯信息****************" << count << std::endl;
//     lightFile << "信号灯数量:" << count << std::endl;
//     std::cout << "接收到信号灯信息数量为:" << count << std::endl;
//     lightFile << "name:" << msg_SPAT.name.c_str() << std::endl;
//     size_t moy = 0; // minute of the year
//     if (msg_SPAT.moy.get() != nullptr)
//     {
//         moy = *msg_SPAT.moy;
//     }
//     size_t timeStamp = 0; /// Time stamp when this message is formed in milliseconds
//     if (msg_SPAT.timeStamp.get() != nullptr)
//     {
//         timeStamp = *msg_SPAT.timeStamp;
//     }
//     lightFile << "moy:" << moy << std::endl;
//     lightFile << "timeStamp:" << timeStamp << std::endl;
//     for (int i = 0; i < count; i++)
//     {
//         std::string intersectionId = msg_SPAT.intersections.list[i].intersectionId;

//         std::cout << "路口ID为:" << intersectionId << std::endl;
//         lightFile << "road id:" << intersectionId << std::endl;
//         // 有多少个相机
//         int phaseCount = msg_SPAT.intersections.list[i].phases.list.size();
//         std::cout << "包含:" << phaseCount << "个相位!" << std::endl;
//         lightFile << "包含:" << phaseCount << "个相位!" << std::endl;
//         for (int j = 0; j < phaseCount; j++)
//         {
//             size_t _id = msg_SPAT.intersections.list[i].phases.list[j].id; // 相位ID
//             lightFile << "-----相位 ID" << _id << std::endl;
//             int phaseStatesCount = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list.size();
//             for (int k = 0; k < phaseStatesCount; k++)
//             {
//                 /**
//                  * 信号灯状态
//                  * 0 相位状态未知或者异常
//                  * 1 相位对应的控制灯为熄灭状态
//                  * 2 对应相位为红闪，通常作为警示灯作用
//                  * 3 红灯
//                  * 4 绿闪
//                  * 5 对应相位为绿灯，且该相位是非专用相位，直行和左转车流存在冲突，相位冲突情况未知或无法确定情况下，绿灯默认状态是permissive-green
//                  * 6 对应相位为绿灯，且该相位是专用相位，车流受信号控制系统保护
//                  * 7 对应相位为黄灯
//                  * 8 对应相位为黄闪，通常作为警示灯作用
//                  */
//                 size_t light = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].light;
//                 /**
//                  * 如果当前该相位状态已开始（未结束），则该数值为0；
//                  * 如果当前该相位状态未开始，则表示当前时刻距离该相位状态下一次开始的时间
//                  */
//                 size_t startTime = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->startTime;
//                 /**
//                  * 表示当前时刻距离该相位状态下一次结束的最短时间（不管当前时刻该相位状态是否开始）
//                  * 对于固定周期配时信号灯，minEndTime应该等于MaxEndTime
//                  */
//                 size_t minEndTime = 0;
//                 if (msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->minEndTime.get() != nullptr)
//                 {
//                     minEndTime = *msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->minEndTime;
//                 }
//                 /**
//                  * 表示当前时刻距离该相位状态下一次结束的最长时间（不管当前时刻该相位状态是否开始）
//                  */
//                 size_t maxEndTime = 0;
//                 if (msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->maxEndTime.get() != nullptr)
//                 {
//                     maxEndTime = *msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->maxEndTime;
//                 }
//                 /**
//                  * 表示当前时刻距离该相位状态下一次结束的估计时间（不管当前时刻该相位状态是否开始）
//                  * 如果该信号灯相位是定周期、固定时长，则该数值表示当前时刻距离该相位状态下一次结束的准确时间
//                  * 如果该信号灯当前相位是非固定配置(感应配时、手动控制等)，则该数值表示预测的结束时间，且预测时间必须在
//                  * minEndTime和maxEndTime之间
//                  * 对于该相位只有固定一种相位状态时（常绿、黄闪灯），应将该相位状态的linklyEndTime设置为36000
//                  */
//                 size_t likelyEndTime = msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->likelyEndTime;
//                 // 置信水平
//                 size_t timeConfidence = 0;
//                 if (msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->timeConfidence.get() != nullptr)
//                 {
//                     timeConfidence = *msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->timeConfidence;
//                 }
//                 /**
//                  * 如果当前该相位状态已开始（未结束），则该数值表示当前时刻距离该相位状态下一次开始的估计
//                  * 时长；如果当前该相位状态未开始，则表示当前时刻距离该相位状态第二次开始的时间。通常用在一些
//                  *  经济驾驶模式（ECO Drive）等相关的应用中。
//                  */
//                 size_t nextStartUTCTime = 0;
//                 if (msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->nextStartTime.get() != nullptr)
//                 {
//                     nextStartUTCTime = *msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->nextStartTime;
//                 }
//                 /**
//                  *如果当前该相位状态已开始（未结束），则该数值表示该相位状态下一次开始后的持续时长；如果
//                  *当前该相位状态未开始，则表示该相位状态第二次开始后的持续时长。与nextStartTime配合使用，通
//                  *常用在一些经济驾驶模式（ECO Drive）等相关的应用中
//                  */
//                 size_t nextDuration = 0;
//                 if (msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->nextDuration.get() != nullptr)
//                 {
//                     nextDuration = *msg_SPAT.intersections.list[i].phases.list[j].phaseStates.list[k].timing->counting->nextDuration;
//                 }

//                 lightFile << "light status:" << light << "\t";
//                 lightFile << "startTime:" << startTime << "\t";
//                 lightFile << "minEndTime:" << minEndTime << "\t";
//                 lightFile << "maxEndTime:" << maxEndTime << "\t";
//                 lightFile << "likelyEndTime:" << likelyEndTime << "\t";
//                 lightFile << "timeConfidence:" << timeConfidence << "\t";
//                 lightFile << "nextStartUTCTime:" << nextStartUTCTime << "\t";
//                 lightFile << "nextDuration:" << nextDuration << "\n";
//             }
//         }
//     }
//     lightFile << "***************完成本次信号灯信息输入****************" << count << std::endl;
// }

// void V2x::SSM2Probuf(const Ivics::SSM &msg_SSM, RoadsideMsgVec *msgvec)
// {
//     size_t partlen = msg_SSM.participants->list.size(); // 交通参与者数量
//     std::cout << "------获取交通参与者数量：" << partlen << std::endl;
//     if (partlen > 0)
//     {
//         for (size_t i = 0; i < partlen; i++)
//         {
//             double x = msg_SSM.sensorPos.x;                          // 传感器位置 x 坐标
//             double y = msg_SSM.sensorPos.y;                          // 传感器位置 y 坐标
//             double px = x + msg_SSM.participants->list[i].ptc.pos.x; // 感知物墨卡托x坐标
//             double py = y + msg_SSM.participants->list[i].ptc.pos.y; // 感知物墨卡托y坐标
//             if (abs(px) < 2000 && abs(py) < 2000)                    // 2公里以内值才算合法值
//             {

//                 /**
//                  * 坐标转换，墨卡托坐标转经纬度，经纬度转高斯
//                  * 按照高铁新城路侧定义的坐标偏移量计算
//                  */
//                 // 1 墨卡托坐标转经纬度

//                 coorconv::WGS84Corr wgs;
//                 wgs.lon = 120;
//                 wgs.lat = 31;
//                 coorconv::UTMCoor utm;
//                 int zone1 = std::floor((wgs.lon + 180.0) / 6.0) + 1;
//                 utm.x = px + offset[0];
//                 utm.y = py + offset[1];
//                 // std::cout << "------before trasfor px:" << std::setprecision(10) << px << std::endl;
//                 // std::cout << "------before trasfor py:" << std::setprecision(10) << py << std::endl;
//                 // std::cout << "------ utm.x:" << std::setprecision(10) << utm.x << std::endl;
//                 // std::cout << "------ utm.y:" << std::setprecision(10) << utm.y << std::endl;
//                 // std::cout << "------before trasfpr wgs.lat:" << std::setprecision(10) << wgs.lat << std::endl;
//                 // std::cout << "------before trasfpr wgs.lon:" << std::setprecision(10) << wgs.lon << std::endl;
//                 coorconv::UTMXYToLatLon(utm, zone1, false, wgs);
//                 // std::cout << "------after trasfpr wgs.lat:" << std::setprecision(10) << wgs.lat << std::endl;
//                 // std::cout << "------after trasfpr wgs.lon:" << std::setprecision(10) << wgs.lon << std::endl;
//                 // 2 经纬度转高斯坐标
//                 ZtGeographyCoordinateTransform ztTran;
//                 double guss_x, guss_y;
//                 ztTran.BL2XY(wgs.lat, wgs.lon, guss_y, guss_x);
//                 // std::cout << "------ trasfpr guss_x------" << std::setprecision(10) << guss_x << std::endl;
//                 // std::cout << "------ trasfpr guss_y------" << std::setprecision(10) << guss_y << std::endl;
//                 // 长、宽、高
//                 double len = (msg_SSM.participants->list[i].ptc.size.length) * 100; // 长度 单位 m 换算成cm
//                 double height = 0.0;                                                // 高度 单位 m
//                 if (msg_SSM.participants->list[i].ptc.size.height.get() != nullptr)
//                 {
//                     height = (*msg_SSM.participants->list[i].ptc.size.height) * 100; // 换算成cm
//                 }
//                 double width = (msg_SSM.participants->list[i].ptc.size.width) * 100; // 宽度 单位 m 换算成cm
//                 double hudu = msg_SSM.participants->list[i].ptc.heading;             // 航向角 其值以正北方向为0点顺时针增加
//                 // double heading = ((hudu*180.0)/PI)/1.25;
//                 double heading = ((PI * 0.5 - hudu) * 180.0) / PI;
//                 int ptc_type = msg_SSM.participants->list[i].ptc.ptcType; // 交通参与者类型：0 未知 1 机动车 2 非机动车 3 行人 4 rsu自身
//                 // 类型转换 转换成云定义的数据： 0 车辆 1 行人 2 骑车人 3 其他
//                 int tran_ptc_type = convertType(ptc_type);            // 默认3
//                 long ptcId = msg_SSM.participants->list[i].ptc.ptcId; // RSU设置的临时ID，0是RSU本身，1...255代表RSU检测到的参与者，RSU中不同参与者的ptcId必须唯一，对应objectid

//                 /*
//                  * // 来源 0 未知来源 1 RSU自身信息 2 来源参与者自身v2x广播信息 3 来源于视频传感器
//                  *   4 来源于微波雷达传感器 5 来源于地磁线圈传感器 6 来源于激光雷达传感器
//                  *   7 融合结果
//                  */
//                 /* size_t source = msg_SSM.participants->list[i].ptc.source;
//                  std::string id;
//                  if (msg_SSM.participants->list[i].ptc.id.get() != nullptr)
//                  {
//                      id = *msg_SSM.participants->list[i].ptc.id;
//                  }
//                   */
//                 size_t secMark = msg_SSM.participants->list[i].ptc.secMark; // utc时间
//                 double speed = msg_SSM.participants->list[i].ptc.speed;     // 速度
//                 /* double angle = 0.0;                                     // 方向盘转向角
//                 double pz = 0.0;                                        // 感知物墨卡托y坐标
//                 if (msg_SSM.participants->list[i].ptc.angle.get() != nullptr)
//                 {
//                     angle = *msg_SSM.participants->list[i].ptc.angle.get();
//                 }
//                 if (msg_SSM.participants->list[i].ptc.pos.z.get() != nullptr)
//                 {
//                     pz = *msg_SSM.participants->list[i].ptc.pos.z.get();
//                 } */
//                 // 转换成proto格式文件
//                 RoadsideMsg *fmsg;
//                 fmsg = msgvec->add_rvecmsg();
//                 fmsg->set_type(tran_ptc_type);
//                 fmsg->set_objectid(ptcId);
//                 fmsg->set_lat(wgs.lat);
//                 fmsg->set_lon(wgs.lon);
//                 fmsg->set_x(guss_x);
//                 fmsg->set_y(guss_y);
//                 fmsg->set_yaw(heading);
//                 fmsg->set_velocity(speed);
//                 fmsg->set_power(100);
//                 fmsg->set_finish(false);
//                 fmsg->set_height(height);
//                 fmsg->set_len(len);
//                 fmsg->set_width(width);
//                 fmsg->set_timestamp(secMark);
//             }
//         }
//     }
// }

bool compareData(double lastX, double lastY, double currentX, double currentY, double lastYaw, double currentYaw, double disT, int degT)
{
    // std::cout << "lastx::" << std::setprecision(10) << lastX << std::endl;
    // std::cout << "lastY::" << std::setprecision(10) << lastY << std::endl;
    // std::cout << "currentX::" << std::setprecision(10) << currentX << std::endl;
    // std::cout << "currentY::" << std::setprecision(10) << currentY << std::endl;
    // std::cout << "lastYaw::" << lastYaw << std::endl;
    // std::cout << "currentYaw::" << currentYaw << std::endl;
    double dis = sqrt(pow(currentX - lastX, 2)) + sqrt(pow(currentY - lastY, 2));
    double angle = abs(currentYaw - lastYaw);
    // std::cout << "dis::" << dis << std::endl;
    // std::cout << "angle::" << angle << std::endl;
    return dis <= disT && angle > degT;
}

/*count x and y distance*/
// by syp// 2D的坐标变换公式，已知全局坐标，车辆坐标系远点在全局坐标系中的坐标，求改点在车辆坐标系中的位置
// 坐标系都为笛卡尔坐标系，xy为右上
// double dX0, double dY0 新坐标原点在旧坐标系中的位置
// double dPhi0为原有X轴在旧坐标系中的角度
void V2x::CoordTran2DForNew0INOld(double &dX, double &dY, double &dPhi, double dX0, double dY0, double dPhi0)
{
    double xTemp, yTemp;
    // 坐标平移
    xTemp = dX - dX0;
    // 终点转换后的坐标X
    yTemp = dY - dY0;
    // 终点转换后的坐标Y
    // 坐标旋转
    dPhi0 = -dPhi0;
    dX = xTemp * cos(dPhi0) - yTemp * sin(dPhi0);
    dY = xTemp * sin(dPhi0) + yTemp * cos(dPhi0);
    dPhi = dPhi + dPhi0; // 终点转换后的角度
}

void V2x::excludeOwnSelfVehicle(RoadsideMsgVec *roadSideVec, SelfVehicle &selfVehicle)
{
    std::ofstream ssmFile;
    ssmFile.open("../logs/ssm_out.txt", std::ios::out | std::ios::app);

    // write data to file
    // 遍历路侧数据
    if (roadSideVec->mutable_rvecmsg() != nullptr)
    {

        bool result = false;
        for (auto it = roadSideVec->mutable_rvecmsg()->begin(); it != roadSideVec->mutable_rvecmsg()->end();)
        {
            const RoadsideMsg &fmsg = *it;
            int Type = fmsg.type();         // 目标类型
            int ObjectID = fmsg.objectid(); // 编号

            double roadX = fmsg.x(); // 高斯坐标x
            double roadY = fmsg.y(); // 高斯坐标y
            double raodYaw = 0.0;    // 航向角
            // 路测检测到有效感知物，并且类型是车时，才进行剔除自车逻辑处理
            if (ObjectID > 0)
            {
                double dis = sqrt(pow(roadX - selfVehicle.gaussX, 2)) + sqrt(pow(roadY - selfVehicle.gaussY, 2));

                if (dis < dis_thre)
                {
                    result = true;
                    ssmFile << "*******************remove******************" << std::endl;
                    ssmFile << " ObjectID::" << std::setprecision(10) << ObjectID << std::endl;
                    ssmFile << " roadX::" << std::setprecision(10) << roadX << std::endl;
                    ssmFile << " roadY::" << std::setprecision(10) << roadY << std::endl;
                    ssmFile << " selfVehicle.gaussX::" << std::setprecision(10) << selfVehicle.gaussX << std::endl;
                    ssmFile << " selfVehicle.gaussY::" << std::setprecision(10) << selfVehicle.gaussY << std::endl;
                    ssmFile << "dis::" << dis << std::endl;
                    ssmFile << "=============================================" << std::endl;
                    // 删除当前元素并更新迭代器
                    it = roadSideVec->mutable_rvecmsg()->erase(it);
                }
                else
                {
                    ++it; // 更新迭代器
                }
                // std::cout << "old roadX::" <<std::setprecision(10) << roadX << std::endl;
                // std::cout << "old roadY::" << std::setprecision(10) <<roadY << std::endl;

                // // 坐标按航向角转换
                // CoordTran2DForNew0INOld(roadX, roadY, raodYaw, selfVehicle.gaussX, selfVehicle.gaussY, selfVehicle.yaw);

                // std::cout << "new roadX::" <<std::setprecision(10) << roadX << std::endl;
                // std::cout << "new roadY::" << std::setprecision(10) <<roadY << std::endl;

                // // 自车在设定范围内，进行剔除
                // if ((abs(roadX) < heng_thre) && (abs(roadY) < zong_thre))
                // {

                //     ssmFile << "*******************remove******************" << ObjectID << std::endl;
                //     roadSideVec.mutable_rvecmsg()->erase(it);
                // }
            }
            else
            {
                ++it; // 更新迭代器
            }
        }
    }

    // ssmFile << "=============================================" << std::endl;
    //  for (auto it = roadSideVec.mutable_rvecmsg()->begin(); it != roadSideVec.mutable_rvecmsg()->end(); ++it)
    //  {
    //      const RoadsideMsg &fmsg = *it;
    //      ssmFile << "after ObjectID:" << fmsg.objectid() << "\n";
    //  }
    // ssmFile << "*****************************" << std::endl;
    ssmFile.close();
}

// 将角度转换为弧度
double V2x::to_radians(double degrees)
{
    return degrees * PI / 180.0;
}
// 根据经纬度计算两点距离（单位：公里）
double V2x::calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = to_radians(lat2 - lat1);
    double dlon = to_radians(lon2 - lon1);
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(to_radians(lat1)) * std::cos(to_radians(lat2)) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = radius_earth * c;
    return distance;
}

void V2x::SSM2Probuf(const Ivics::SSM &msg_SSM, RoadsideMsgVec *msgvec, ParameSct &ps)
{
    size_t partlen = msg_SSM.participants->list.size(); // 交通参与者数量
    std::cout << "\033[1;34m "
              << "*****【info】:  获取原始路测感知物数量::" << partlen << "\033[0m " << std::endl;
    if (partlen > 0)
    {
        RoadsideMsgVec tempVec;
        SelfVehicle selfVehicle;
        miliClock_type tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        int64_t nowts = tp.time_since_epoch().count();
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        std::chrono::seconds secondes = std::chrono::duration_cast<std::chrono::seconds>(duration);
        int second = secondes.count();

        for (size_t i = 0; i < partlen; i++)
        {
            double x = msg_SSM.sensorPos.x;                          // 传感器位置 x 坐标
            double y = msg_SSM.sensorPos.y;                          // 传感器位置 y 坐标
            double px = x + msg_SSM.participants->list[i].ptc.pos.x; // 感知物墨卡托x坐标
            double py = y + msg_SSM.participants->list[i].ptc.pos.y; // 感知物墨卡托y坐标
            double svLat = ps.mySelfVehicle->lat;                    // 自车lat
            double svLon = ps.mySelfVehicle->lon;                    // 自车lon
            /**
             * 坐标转换，墨卡托坐标转经纬度，经纬度转高斯
             * 按照高铁新城路侧定义的坐标偏移量计算
             */
            // 1 墨卡托坐标转经纬度
            coorconv::WGS84Corr wgs;
            wgs.lon = 120;
            wgs.lat = 31;
            coorconv::UTMCoor utm;
            int zone1 = std::floor((wgs.lon + 180.0) / 6.0) + 1;
            utm.x = px + offset[0];
            utm.y = py + offset[1];
            coorconv::UTMXYToLatLon(utm, zone1, false, wgs);
            // 计算自车和感知物之间距离,取pre_max范围内的数据
            double distance = std::abs(calculate_distance(svLat, svLon, wgs.lat, wgs.lon));
            if (distance < pre_max)
            {
                // 2 经纬度转高斯坐标
                ZtGeographyCoordinateTransform ztTran;
                double guss_x, guss_y;
                ztTran.BL2XY(wgs.lat, wgs.lon, guss_y, guss_x);
                // 长、宽、高
                double len = (msg_SSM.participants->list[i].ptc.size.length) * 100; // 长度 单位 m 换算成cm
                double height = 0.0;                                                // 高度 单位 m
                if (msg_SSM.participants->list[i].ptc.size.height.get() != nullptr)
                {
                    height = (*msg_SSM.participants->list[i].ptc.size.height) * 100; // 换算成cm
                }
                double width = (msg_SSM.participants->list[i].ptc.size.width) * 100; // 宽度 单位 m 换算成cm
                double hudu = msg_SSM.participants->list[i].ptc.heading;             // 航向角 其值以正北方向为0点顺时针增加
                // double heading = ((hudu*180.0)/PI)/1.25;
                double current_heading = ((PI * 0.5 - hudu) * 180.0) / PI;
                int ptc_type = msg_SSM.participants->list[i].ptc.ptcType; // 交通参与者类型：0 未知 1 机动车 2 非机动车 3 行人 4 rsu自身
                // 类型转换 转换成云定义的数据： 0 车辆 1 行人 2 骑车人 3 其他
                int tran_ptc_type = convertType(ptc_type);            // 默认3
                long ptcId = msg_SSM.participants->list[i].ptc.ptcId; // RSU设置的临时ID，0是RSU本身，1...255代表RSU检测到的参与者，RSU中不同参与者的ptcId必须唯一，对应objectid
                // size_t current_secMark = msg_SSM.participants->list[i].ptc.secMark; // utc时间
                size_t current_secMark = nowts;
                double speed = msg_SSM.participants->list[i].ptc.speed; // 速度
                auto it = hisMap.find(ptcId);

                if (it != hisMap.end())
                {
                    // std::cout << "**********include key:" << ptcId << std::endl;
                    Object hisObject = it->second;
                    int last_yaw = hisObject.Yaw;
                    long long last_ts = hisObject.Timestamp;
                    double lastX = hisObject.X;
                    double lastY = hisObject.Y;
                    bool result = compareData(lastX, lastY, guss_x, guss_y, last_yaw, current_heading, disT, degT);
                    // std::cout << "compareData result:" << result << std::endl;
                    if (result)
                    {
                        current_heading = last_yaw;
                    }
                    // update hisdat to current data
                    it->second.ObjectID = ptcId;
                    it->second.Type = tran_ptc_type;
                    it->second.Lat = wgs.lat;
                    it->second.Lon = wgs.lon;
                    it->second.X = guss_x;
                    it->second.Y = guss_y;
                    it->second.Yaw = current_heading;
                    it->second.Velocity = speed;
                    it->second.Power = 100;
                    it->second.Finish = false;
                    it->second.Len = len;
                    it->second.Width = width;
                    it->second.Height = height;
                    it->second.Timestamp = current_secMark;
                    timeMap[ptcId] = std::chrono::seconds(second);
                }
                else
                {
                    Object obj;
                    obj.ObjectID = ptcId;
                    obj.Type = tran_ptc_type;
                    obj.Lat = wgs.lat;
                    obj.Lon = wgs.lon;
                    obj.X = guss_x;
                    obj.Y = guss_y;
                    obj.Yaw = current_heading;
                    obj.Velocity = speed;
                    obj.Power = 100;
                    obj.Finish = false;
                    obj.Len = len;
                    obj.Width = width;
                    obj.Height = height;
                    obj.Timestamp = current_secMark;
                    hisMap.emplace(ptcId, obj);
                    timeMap.emplace(ptcId, second);
                }

                // 转换成proto格式文件
                RoadsideMsg *fmsg;
                fmsg = tempVec.add_rvecmsg();
                fmsg->set_type(tran_ptc_type);
                fmsg->set_objectid(ptcId);
                fmsg->set_lat(wgs.lat);
                fmsg->set_lon(wgs.lon);
                fmsg->set_x(guss_x);
                fmsg->set_y(guss_y);
                fmsg->set_yaw(current_heading);
                fmsg->set_velocity(speed);
                fmsg->set_power(100);
                fmsg->set_finish(false);
                fmsg->set_height(height);
                fmsg->set_len(len);
                fmsg->set_width(width);
                fmsg->set_timestamp(current_secMark);
            }
        }
        std::cout << "\033[1;34m "
                  << "*****【info】:  在阈值范围内的感知物数量::" << tempVec.rvecmsg_size() << "\033[0m " << std::endl;
        // excluding ego vehice from roadside ssm data
        if (tempVec.rvecmsg_size() > 0)
        {

            excludeOwnSelfVehicle(&tempVec, *ps.mySelfVehicle);
        }
        // 赋值
        *msgvec = std::move(tempVec);

        // 将数据转存到 ps. send cloud
        // if (msgvec->rvecmsg_size() > 0)
        // {
        //     ObjectsVec msgvecTemp; // 发送给服务器的消息临时变量
        //     for (int i = 0; i < (int)msgvec->rvecmsg_size(); i++)
        //     {
        //         ObjectsVec_ObjectsProto *ptrObjectsProtoTemp = msgvecTemp.add_objmsg();
        //         ptrObjectsProtoTemp->set_type(msgvec->rvecmsg(i).type());
        //         // ptrObjectsProtoTemp->set_objectid(15000000 + i);
        //         ptrObjectsProtoTemp->set_objectid(msgvec->rvecmsg(i).objectid());
        //         ptrObjectsProtoTemp->set_lat(msgvec->rvecmsg(i).lat());
        //         ptrObjectsProtoTemp->set_lon(msgvec->rvecmsg(i).lon());
        //         ptrObjectsProtoTemp->set_x(msgvec->rvecmsg(i).x());
        //         ptrObjectsProtoTemp->set_y(msgvec->rvecmsg(i).y());
        //         ptrObjectsProtoTemp->set_yaw(msgvec->rvecmsg(i).yaw());
        //         ptrObjectsProtoTemp->set_velocity(msgvec->rvecmsg(i).velocity());
        //         ptrObjectsProtoTemp->set_power(100);
        //         ptrObjectsProtoTemp->set_finish(false);
        //         ptrObjectsProtoTemp->set_height(msgvec->rvecmsg(i).height());
        //         ptrObjectsProtoTemp->set_len(msgvec->rvecmsg(i).len());
        //         ptrObjectsProtoTemp->set_width(msgvec->rvecmsg(i).width());
        //         ptrObjectsProtoTemp->set_timestamp(msgvec->rvecmsg(i).timestamp());
        //     }
        //     msgvecTemp.set_sensorid(55);

        //     ps.ptrMutexMsgvec->lock();
        //     (*ps.ptrMsgvec) = msgvecTemp;
        //     (*ps.ptrBRenewMsgvec) = true;
        //     ps.ptrMutexMsgvec->unlock();

        //     std::cout << "------准备发送数据to cloud：" << msgvecTemp.objmsg_size() << std::endl;
        // }

        // clear overtime element
        auto it = timeMap.begin();
        while (it != timeMap.end())
        {
            auto elemTimeInSeconds = it->second.count();
            auto key = it->first;
            auto diff = second - elemTimeInSeconds;
            if (diff > time_threshold)
            {
                it = timeMap.erase(it);
                hisMap.erase(key);
            }
            else
            {
                ++it;
            }
        }
        std::ofstream ssmFile;
        ssmFile.open("../logs/ssm_data.txt", std::ios::out | std::ios::app);
        ssmFile << "hisMap size::" << hisMap.size() << std::endl;
        // write data to file
        for (auto it = hisMap.begin(); it != hisMap.end(); ++it)
        {
            int key = it->first;
            Object value = it->second;
            ssmFile << "key:" << key << std::endl;
            ssmFile << "Timestamp:" << value.Timestamp << "\t";
            ssmFile << "Lat:" << std::setprecision(10) << value.Lat << "\t";
            ssmFile << "Lon:" << std::setprecision(10) << value.Lon << "\t";
            ssmFile << "X:" << std::setprecision(10) << value.X << "\t";
            ssmFile << "Y:" << std::setprecision(10) << value.Y << "\t";
            ssmFile << "Yaw:" << value.Yaw << "\t";
            ssmFile << "Len:" << value.Len << "\t";
            ssmFile << "Width:" << value.Width << "\t";
            ssmFile << "Velocity:" << value.Velocity << "\n";
        }
        ssmFile.close();
    }
}
int V2x::convertType(int type)
{
    switch (type)
    {
    case 0:
    case 4:
        return 3;
        break;
    case 1:
        return 0;
        break;
    case 2:
        return 2;
        break;
    case 3:
        return 1;
        break;

    default:
        return 3;
        break;
    }
}

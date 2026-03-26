#pragma once

#include <iostream>
#include <sstream>
#include <assert.h>
#include <unistd.h>
#include <mutex>
#include <list>
// for thread and timer:
#include <iostream>
// for zmq:
#include <zmq.h>
#include "localization_MapAnalysis.h"
#include "localPlanning.hpp"

// add by syp
#include "uiMsg/ui.pb.h"
#include "serverMsg/decision.pb.h"
#include "serverMsg/msg2vehicle.pb.h"
#include "serverMsg/objects.pb.h"
#include "serverMsg/objectsvec.pb.h"
#include "serverMsg/traffic_light.pb.h"
#include "planningMsg/RoutingMsg.pb.h"
#include "serverMsg/traffic_light.pb.h"
// #include  "mapMsg/TrafficLight.h"
#include "TrafficLight.h"
#include "perception/trafficLightFromPerc.pb.h"

// time
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
// end of by syp

#include "baseData.h"

// predict20230907

struct simplePredictionInfo
{
    vector<cv::Point2d> objectstrajectory;
    vector<int> timestamp;
    vector<bool> isDisapear; // 会出问题
    vector<infopack::ObjectsProto> fata;
    double area = 0;
    bool isUpdate = false;
    bool isMoving = false;
    int life = 0;
};

struct simplePredictionInfoClu
{
    vector<cv::Point2d> objectstrajectory;
    vector<int> timestamp;
    vector<bool> isDisapear; // 会出问题
    vector<prediction::Object> fata;
    double area = 0;
    bool isUpdate = false;
    bool isMoving = false;
    int life = 0;
};

class simplePredict
{
public:
    simplePredict() = default;
    ~simplePredict() {}

    std::vector<infopack::ObjectsProto> simplePrediction(const std::vector<infopack::ObjectsProto> &objects, const pc::Imu &imu)
    {
        std::cout << "simple prediction input size" << objects.size() << std::endl;
        vector<infopack::ObjectsProto> predictionObjects;
        int movingNums = 0;
        // cout<< "imu.yaw:"<<imu.yaw()<<endl;
        for (int i = 0; i < objects.size(); i++) // 更新原有障碍物位置，并在障碍物列表中添加新的障碍物
        {
            infopack::ObjectsProto obj = objects[i];

            if (fabs(objects[i].velocity()) < 0.2) // 速度小的滤掉
            {
                continue;
            }
            cv::Point2d temp2;
            temp2.y = objects[i].lat();
            temp2.x = objects[i].lon();
            double yaw = M_PI / 2 - objects[i].yaw() * M_PI / 180.;

            cv::Point2d gauss2;
            gaussConvert(temp2.x, temp2.y, gauss2.x, gauss2.y); // 经纬度转高斯坐标
            // cout<<i<<std::setprecision(10)<<" gauss2.xx: "<< gauss2.x<<"  gauss2.yy: "<< gauss2.y<<endl;

            double veX, veY, veYaw;
            veX = imu.gaussy(); // 自车的位置
            veY = imu.gaussx();
            veYaw = M_PI / 2 - imu.yaw() * M_PI / 180.;
            //  cout<< "veYaw.yaw:"<<veYaw<<endl;
            if (getDistance_2(veY, veX, gauss2.x, gauss2.y) > 100 * 100) // 100米以外的过滤
            {
                continue;
            }
            CoordTran2DForNew0INOld(gauss2.y, gauss2.x, yaw, veX, veY, veYaw); //  高斯坐标转换车当前坐标
            // cout<<i<<" gauss2.y: "<< gauss2.y<<" objects[i]: "<< objects[i].y()<<endl;

            if ((gauss2.x < 1.5 && gauss2.x > -1.5 && gauss2.y < 0) || gauss2.y < -50 || (abs(objects[i].yaw() - imu.yaw()) > 150 && abs(objects[i].yaw() - imu.yaw()) < 210 && abs(gauss2.x) > 15)) // 车正后方的滤掉,左右1.5米;车后方50以外的滤掉;左右15米以外的，逆向的滤掉
            {
                continue;
            }

            // if (obj.type() != 0 && obj.objectid() != 1)
            // cout<<"objects[i].type(): "<<objects[i].type()<<endl;
            // cout<<i<<"'s"<<"objects[i].id(): "<<objects[i].objectid()<<endl;
            // if (objects[i].velocity() > 20){
            //     cout << i << "'s" << "objects[i].velocity(): " << objects[i].velocity() << endl;
            //     }
            // objects[i].
            // cout<<i<<"'s "<<std::setprecision(10)<<
            //                                  "  objects[i].x: "<<objects[i].x()<<
            //                                   "  objects[i].y: "<<objects[i].y()<<
            //                                  endl;
            //      cout<<i<<"'s "<<"objects[i].id: "<< objects[i].objectid()<<endl;
            // cout<<i<<"'s "<<"objects[i].yaw(): "<< objects[i].yaw()<<
            //                                  "  objects[i].lat(): "<<objects[i].lat()<<
            //                                   "  objects[i].lon(): "<<objects[i].lon()<<
            //                                   " imu.yaw:"<<imu.yaw()<<
            //                                   setprecision(13)<<" imu.time():"<<imu.time()<<
            //                                   " objects[i].timestamp():"<<
            //                                   objects[i].timestamp()<<endl;

            //      cout<<i<<"'s "<<"objects[i].id: "<< objects[i].objectid()<<endl;
            // if(objects[i].type()!=0)
            // {
            // struct timespec time1 = {0, 0};
            // clock_gettime(CLOCK_REALTIME, &time1);
            // cout<<"real time: "<<time1.tv_sec<<'.'<<time1.tv_nsec<<endl;
            int timestamp; // ms
            timestamp = objects[i].timestamp();
            if (timestamp > newestTime)
                newestTime = timestamp;
            // cout<<"time: "<<timestamp<<endl;
            cv::Point2d temp1;
            temp1.y = objects[i].lat();
            temp1.x = objects[i].lon();
            cv::Point2d gauss;
            gaussConvert(temp1.x, temp1.y, gauss.x, gauss.y);
            double objectArea = (obj.len() / 100.) * (obj.width() / 100.);
            bool isNew = true;
            // cout<<YELLOW<<historyObjects.size()<<endl;
            // std::cout<<YELLOW<<"object position "<<gauss.x<<" "<<gauss.y<<RESET<<std::endl;
            // double temp_x=gauss.x;
            // double temp_y=gauss.y;
            for (int j = 0; j < historyObjects.size(); j++)
            {
                //    cout<<RED<<"historyObjects.size()!"<<historyObjects.size()<<std::endl;
                // std::cout<<YELLOW<<"       related object position "<<historyObjects[j].objectstrajectory[0].x-gauss.x<<" "<<historyObjects[j].objectstrajectory[0].y-gauss.y<<RESET<<std::endl;
                // cout<<"       abs object position "<<historyObjects[j].objectstrajectory[0].x<<" "<<historyObjects[j].objectstrajectory[0].y<<endl;
                // if(historyObjects[j].isUpdate)
                // cout<<RED<<"UPDATE!";
                // else
                // cout<<RED<<"NO UPDATE !!";
                // temp_x=historyObjects[j].objectstrajectory[0].x;
                // temp_y=historyObjects[j].objectstrajectory[0].y;
                double dis = getDistance(gauss.x, gauss.y, historyObjects[j].objectstrajectory[0].x, historyObjects[j].objectstrajectory[0].y);
                int32_t ID = historyObjects[j].fata[0].objectid();
                //  cout<<j<<"'s "<<"ID:  "<<ID<<endl;
                // cout<<"distance:  "<<setprecision(8)<<dis<<endl;
                // if (dis <5 &&( abs(objectArea - historyObjects[j].area) < 3)||historyObjects[max(0,j-1)].isMoving||historyObjects[max(0,j-4)].isMoving||historyObjects[max(0,j-8)].isMoving||historyObjects[max(0,j-10)].isMoving||historyObjects[max(0,j-2)].isMoving)
                if (dis < 5 && abs(objectArea - historyObjects[j].area) < 3 && ID == objects[i].objectid()) // 加入ID判断
                // if ( ID ==objects[i].objectid() )  && ID ==objects[i].objectid()
                {
                    isNew = false;
                    historyObjects[j].isUpdate = true;
                    historyObjects[j].objectstrajectory.insert(historyObjects[j].objectstrajectory.begin(), gauss);
                    historyObjects[j].timestamp.insert(historyObjects[j].timestamp.begin(), timestamp);
                    historyObjects[j].fata.insert(historyObjects[j].fata.begin(), obj);
                    historyObjects[j].isDisapear.insert(historyObjects[j].isDisapear.begin(), false);
                    historyObjects[j].area = objectArea;
                    while (historyObjects[j].timestamp[0] - historyObjects[j].timestamp.back() > lifeCyele)
                    {
                        historyObjects[j].objectstrajectory.pop_back();
                        historyObjects[j].timestamp.pop_back();
                        historyObjects[j].fata.pop_back();
                        historyObjects[j].isDisapear.pop_back();
                    }
                    historyObjects[j].life = historyObjects[j].timestamp[0] - historyObjects[j].timestamp.back();
                    cout << BLUE << "object life: " << historyObjects[j].life << RED << "   ID: " << j << YELLOW << "   Distancec: " << dis << endl;
                    if (dis > 0.02)
                    {
                        historyObjects[j].isMoving = true;
                        movingNums++;
                    }
                }
                else
                {
                    // if(dis<4)
                    // cout<<GREEN<<"object life: "<<historyObjects[j].life<<RED<<"   ID: "<<j<<YELLOW<<"   Distancec: "<<dis<<endl;
                }
            }
            if (isNew)
            {
                // cout<<GREEN<<"get new!!!"<<endl;
                simplePredictionInfo temp;
                temp.objectstrajectory.push_back(gauss);
                temp.timestamp.push_back(timestamp);
                temp.isDisapear.push_back(false);
                temp.area = objectArea;
                temp.isUpdate = true;
                temp.fata.push_back(obj);
                // temp.fata.insert(temp.fata.begin(), obj);
                historyObjects.push_back(temp);
            }
            // struct simplePredictionInfo
            // {
            //     vector<cv::Point2d> objectstrajectory;
            //     vector<int> timestamp;
            //     vector<bool> isDisapear; // 会出问题
            //     vector<infopack::ObjectsProto> fata;
            //     double area = 0;
            //     bool isUpdate = false;
            //     bool isMoving = false;
            //     int life = 0;
            // };
            // }
        }
        cout << GREEN << "movingNums: " << movingNums << RESET << endl;
        int i = 0;
        //  cout<<BLUE<<"historyObjects.size()!"<<historyObjects.size()<<endl;
        while (i < historyObjects.size()) // 删除连续4帧没有出现的物体
        {
            // cout<<BLUE<<"in the delete:"<<i<<"  of  "<<historyObjects.size()<<endl;
            // if(historyObjects[i].isUpdate)
            // cout<<YELLOW<<"UPDATE !!"<<endl;
            // else
            // cout<<YELLOW<<"NO UPDATE!!"<<endl;
            if (historyObjects[i].isUpdate == false)
            {
                // cout<<"FALSE!!!"<<endl;
                historyObjects[i].isDisapear.insert(historyObjects[i].isDisapear.begin(), true);
                if (historyObjects[i].isDisapear.size() > 20)
                {
                    if (historyObjects[i].isDisapear[1] == true && historyObjects[i].isDisapear[2] == true && historyObjects[i].isDisapear[3] == true)
                    {
                        auto iter = historyObjects.erase(std::begin(historyObjects) + i);
                        cout << BLUE << "erase!" << endl;
                    }
                    else
                    {
                        historyObjects[i].isDisapear.pop_back();
                        i++;
                        // cout<<BLUE<<i<<"  :  "<<historyObjects[i].isDisapear.size()<<endl;
                    }
                }
            }
            else
            {
                historyObjects[i].isUpdate = false;
                // cout<<"@@@@@FALSE!!!"<<endl;
                i++;
            }
        }

        for (int j = 0; j < historyObjects.size(); j++)
        {
            vector<cv::Point2d> prid;
            // if (historyObjects[j].isMoving)
            if (1)
            {
                // cout<<YELLOW<<"moving objects history size:"<<historyObjects[j].objectstrajectory.size()<<endl;
                // if (historyObjects[j].objectstrajectory.size() == 2)
                // {
                //     cv::Point2d x1, x2;
                //     x1.y = historyObjects[j].fata[0].lat();
                //     x1.x = historyObjects[j].fata[0].lon();
                //     x2.y = historyObjects[j].fata[1].lat();
                //     x2.x = historyObjects[j].fata[1].lon();
                //     prid = Prid2(x1, x2);
                //     cout<<YELLOW<<"2 set prediction!!!"<<endl;
                // }
                // if (historyObjects[j].objectstrajectory.size() > 2 && historyObjects[j].life>0)
                // {
                //     // cv::Point2d x1, x2, x3;
                //     // x1.y = historyObjects[j].fata[0].lat();
                //     // x1.x = historyObjects[j].fata[0].lon();
                //     // x2.y = historyObjects[j].fata[2].lat();
                //     // x2.x = historyObjects[j].fata[2].lon();
                //     // x3.y = historyObjects[j].fata[4].lat();
                //     // x3.x = historyObjects[j].fata[4].lon();
                //     // prid = Prid3(x1, x2, x3);
                //     // cout<<YELLOW<<"3 set prediction!!!"<<endl;
                //     prid=Prid(historyObjects[j]);
                // }
                if (historyObjects[j].life > 0)
                    prid = Prid(historyObjects[j]);
                for (int k = 0; k < prid.size(); k++)
                {
                    infopack::ObjectsProto tempPrid = historyObjects[j].fata[0];
                    double lat, lon;

                    gaussConvert(prid[k].y, prid[k].x, lat, lon);
                    tempPrid.set_x(lon);
                    tempPrid.set_y(lat);
                    tempPrid.set_lon(prid[k].x);
                    tempPrid.set_lat(prid[k].y);
                    predictionObjects.push_back(tempPrid);
                }
                historyObjects[j].isMoving = false;
            }
        }
        // std::cout <<GREEN<< "prediction NUMS :" << predictionObjects.size() << std::endl;
        std::cout << GREEN << "historyObjects.size():" << historyObjects.size() << std::endl;
        for (int k = 0; k < historyObjects.size(); k++)
        {
            // cout<<YELLOW<<k<<" : "<<historyObjects[k].objectstrajectory.size()<<endl;
            // for(int l=0;l<historyObjects[k].objectstrajectory.size();l++)
            // {
            //     cout<<YELLOW<<historyObjects[k].objectstrajectory[l].x<<" "<<historyObjects[k].objectstrajectory[l].y<<endl;
            // }
        }
        return predictionObjects;
    };

    // prediction proto重写输入为激光聚类识别的预测
    vector<prediction::Object> simplePredictionClu(prediction::ObjectList &objects, const pc::Imu &imu)
    {
        std::cout << "clu simple prediction input size" << objects.object_size() << std::endl;
        vector<prediction::Object> predictionObjects;
        int movingNums = 0;
        // cout << "imu.yaw:" << imu.yaw() << endl;
        // for (int i = 0; i < objects.object_size(); i++) // 更新原有障碍物位置，并在障碍物列表中添加新的障碍物

        for (auto &object : objects.object())
        {
            // if (object.type() != 0 || object.type() != 1 || object.type() != 2 || object.type() != 3 || object.predictpoint(0).vx() < 0.2)
            // {
            //     continue;
            // }

            double yaw = M_PI / 2 - object.predictpoint(0).vy();
            // std::cout << "object.yaw" << yaw << "velocity:" << object.predictpoint(0).vy() << "id: " << object.trackid() << " chang " << object.l() << " kaun " << object.w() << "type" << object.type() << std::endl;
            // cv::Point2d gauss2;
            // gaussConvert(temp2.x, temp2.y, gauss2.x, gauss2.y);//经纬度转高斯坐标
            // // cout<<i<<std::setprecision(10)<<" gauss2.xx: "<< gauss2.x<<"  gauss2.yy: "<< gauss2.y<<endl;

            double veX, veY, veYaw;
            veX = imu.gaussy(); // 自车的位置
            veY = imu.gaussx();
            veYaw = M_PI / 2 - imu.yaw() * M_PI / 180.;
            cv::Point2d gauss, tempLocal;
            gauss.x = object.predictpoint(0).x();
            gauss.y = object.predictpoint(0).y();
            tempLocal.x = gauss.x;
            tempLocal.y = gauss.y;
            CoordTran2DForNew0INOld(tempLocal.y, tempLocal.x, yaw, veX, veY, veYaw); //  高斯坐标转换车当前坐标
            // cout << " tempLocal.y: " << tempLocal.y << " tempLocalx: " << tempLocal.x << endl;

            if ((tempLocal.x < 1.5 && tempLocal.x > -1.5 && tempLocal.y < 0) || tempLocal.y < -50) // 车正后方的滤掉,左右1.5米;车后方50以外的滤掉
            {
                continue;
            }
            // gaussConvert(temp1.x, temp1.y, gauss.x, gauss.y);
            double objectArea = object.w() * object.h();
            bool isNew = true;

            for (int j = 0; j < historyObjectsClu.size(); j++)
            {
                //    cout<<RED<<"historyObjectsClu.size()!"<<historyObjectsClu.size()<<std::endl;
                // std::cout<<YELLOW<<"       related object position "<<historyObjectsClu[j].objectstrajectory[0].x-gauss.x<<" "<<historyObjectsClu[j].objectstrajectory[0].y-gauss.y<<RESET<<std::endl;
                // cout<<"       abs object position "<<historyObjectsClu[j].objectstrajectory[0].x<<" "<<historyObjectsClu[j].objectstrajectory[0].y<<endl;
                // if(historyObjectsClu[j].isUpdate)
                // cout<<RED<<"UPDATE!";
                // else
                // cout<<RED<<"NO UPDATE !!";
                // temp_x=historyObjectsClu[j].objectstrajectory[0].x;
                // temp_y=historyObjectsClu[j].objectstrajectory[0].y;
                double dis = getDistance(object.predictpoint(0).x(), object.predictpoint(0).y(), historyObjectsClu[j].objectstrajectory[0].x, historyObjectsClu[j].objectstrajectory[0].y);
                int ID = historyObjectsClu[j].fata[0].trackid();
                //  cout<<j<<"'s "<<"ID:  "<<ID<<endl;
                // cout<<"distance:  "<<setprecision(8)<<dis<<endl;
                // if (dis <5 &&( abs(objectArea - historyObjectsClu[j].area) < 3)||historyObjectsClu[max(0,j-1)].isMoving||historyObjectsClu[max(0,j-4)].isMoving||historyObjectsClu[max(0,j-8)].isMoving||historyObjectsClu[max(0,j-10)].isMoving||historyObjectsClu[max(0,j-2)].isMoving)
                if (dis < 5 && abs(objectArea - historyObjectsClu[j].area) < 3 && ID == object.trackid()) // 加入ID判断
                // if ( ID ==objects[i].objectid() )  && ID ==objects[i].objectid()
                {
                    isNew = false;
                    historyObjectsClu[j].isUpdate = true;
                    historyObjectsClu[j].objectstrajectory.insert(historyObjectsClu[j].objectstrajectory.begin(), gauss);
                    historyObjectsClu[j].timestamp.insert(historyObjectsClu[j].timestamp.begin(), imu.time() * 1000);
                    historyObjectsClu[j].fata.insert(historyObjectsClu[j].fata.begin(), object);
                    historyObjectsClu[j].isDisapear.insert(historyObjectsClu[j].isDisapear.begin(), false);
                    historyObjectsClu[j].area = objectArea;
                    while (historyObjectsClu[j].timestamp[0] - historyObjectsClu[j].timestamp.back() > lifeCyele)
                    {
                        historyObjectsClu[j].objectstrajectory.pop_back();
                        historyObjectsClu[j].timestamp.pop_back();
                        historyObjectsClu[j].fata.pop_back();
                        historyObjectsClu[j].isDisapear.pop_back();
                    }
                    historyObjectsClu[j].life = historyObjectsClu[j].timestamp[0] - historyObjectsClu[j].timestamp.back();
                    // cout<<BLUE<<"object life: "<<historyObjectsClu[j].life<<RED<<"   ID: "<<j<<YELLOW<<"   Distancec: "<<dis<<endl;
                    if (dis > 0.02)
                    {
                        historyObjectsClu[j].isMoving = true;
                        movingNums++;
                    }
                }
                else
                {
                    // if(dis<4)
                    // cout<<GREEN<<"object life: "<<historyObjectsClu[j].life<<RED<<"   ID: "<<j<<YELLOW<<"   Distancec: "<<dis<<endl;
                }
            }
            if (isNew)
            {
                // cout<<GREEN<<"get new!!!"<<endl;
                simplePredictionInfoClu temp;
                temp.objectstrajectory.push_back(gauss);
                temp.timestamp.push_back(imu.time() * 1000);
                temp.isDisapear.push_back(false);
                temp.area = objectArea;
                temp.isUpdate = true;
                temp.fata.push_back(object);
                // temp.fata.insert(temp.fata.begin(), obj);
                historyObjectsClu.push_back(temp);
            }
        }
        cout << GREEN << "movingNums: " << movingNums << RESET << endl;
        int i = 0;
        //  cout<<BLUE<<"historyObjectsClu.size()!"<<historyObjectsClu.size()<<endl;
        while (i < historyObjectsClu.size()) // 删除连续4帧没有出现的物体
        {
            // cout<<BLUE<<"in the delete:"<<i<<"  of  "<<historyObjectsClu.size()<<endl;
            // if(historyObjectsClu[i].isUpdate)
            // cout<<YELLOW<<"UPDATE !!"<<endl;
            // else
            // cout<<YELLOW<<"NO UPDATE!!"<<endl;
            if (historyObjectsClu[i].isUpdate == false)
            {
                // cout<<"FALSE!!!"<<endl;
                historyObjectsClu[i].isDisapear.insert(historyObjectsClu[i].isDisapear.begin(), true);
                if (historyObjectsClu[i].isDisapear.size() > 20)
                {
                    if (historyObjectsClu[i].isDisapear[1] == true && historyObjectsClu[i].isDisapear[2] == true && historyObjectsClu[i].isDisapear[3] == true)
                    {
                        auto iter = historyObjectsClu.erase(std::begin(historyObjectsClu) + i);
                        cout << BLUE << "erase!" << endl;
                    }
                    else
                    {
                        historyObjectsClu[i].isDisapear.pop_back();
                        i++;
                        // cout<<BLUE<<i<<"  :  "<<historyObjectsClu[i].isDisapear.size()<<endl;
                    }
                }
            }
            else
            {
                historyObjectsClu[i].isUpdate = false;
                // cout<<"@@@@@FALSE!!!"<<endl;
                i++;
            }
        }

        for (int j = 0; j < historyObjectsClu.size(); j++)
        {
            vector<cv::Point2d> prid;
            // if (historyObjectsClu[j].isMoving)
            if (1)
            {
                // cout<<YELLOW<<"moving objects history size:"<<historyObjectsClu[j].objectstrajectory.size()<<endl;
                // if (historyObjectsClu[j].objectstrajectory.size() == 2)
                // {
                //     cv::Point2d x1, x2;
                //     x1.y = historyObjectsClu[j].fata[0].lat();
                //     x1.x = historyObjectsClu[j].fata[0].lon();
                //     x2.y = historyObjectsClu[j].fata[1].lat();
                //     x2.x = historyObjectsClu[j].fata[1].lon();
                //     prid = Prid2(x1, x2);
                //     cout<<YELLOW<<"2 set prediction!!!"<<endl;
                // }
                // if (historyObjectsClu[j].objectstrajectory.size() > 2 && historyObjectsClu[j].life>0)
                // {
                //     // cv::Point2d x1, x2, x3;
                //     // x1.y = historyObjectsClu[j].fata[0].lat();
                //     // x1.x = historyObjectsClu[j].fata[0].lon();
                //     // x2.y = historyObjectsClu[j].fata[2].lat();
                //     // x2.x = historyObjectsClu[j].fata[2].lon();
                //     // x3.y = historyObjectsClu[j].fata[4].lat();
                //     // x3.x = historyObjectsClu[j].fata[4].lon();
                //     // prid = Prid3(x1, x2, x3);
                //     // cout<<YELLOW<<"3 set prediction!!!"<<endl;
                //     prid=Prid(historyObjectsClu[j]);
                // }
                if (historyObjectsClu[j].life > 0)
                    prid = Prid4(historyObjectsClu[j]);
                for (int k = 0; k < prid.size(); k++)
                {
                    prediction::Object tempPrid = historyObjectsClu[j].fata[0];
                    double lat, lon;

                    gaussConvert(prid[k].y, prid[k].x, lat, lon);

                    //    prediction::PredictPoint* temp;
                    //    temp=tempPrid.mutable_predictpoint(0) ;
                    //     (*temp).set_x(lon);
                    (tempPrid.mutable_predictpoint(0))->set_x(lon);
                    (*tempPrid.mutable_predictpoint(0)).set_y(lat);
                    tempPrid.set_h(prid[k].x);
                    tempPrid.set_z(prid[k].y);
                    predictionObjects.push_back(tempPrid);
                }

                historyObjectsClu[j].isMoving = false;

                // prediction::Object *object = predictionTemp.add_object();
                // // for (int i =0; i <=0; i++)
                // // {
                //     prediction::PredictPoint *predictPoint = object->add_predictpoint();

                //     // predictPoint->set_x(3478542.032+((imu.time()-(int) imu.time()))*10);
                //     predictPoint->set_x(3478542.032);
                //     predictPoint->set_y(559627.015);
                //     predictPoint->set_vx(1);
                //     // predictPoint->set_vy(105.16);
                //     predictPoint->set_vy(imu.yaw());
                // // }
                // object->set_z(31.42749349);
                // // object->set_h(120.62714822 +((imu.time()-(int) imu.time()))*0.0001);
                // object->set_h(120.62714822);
                // object->set_l(10);
                // object->set_w(5);
                // object->set_type(1);
                // object->set_trackid(1);
            }
        }
        // std::cout <<GREEN<< "prediction NUMS :" << predictionObjects.size() << std::endl;
        std::cout << GREEN << "historyObjectsClu.size():" << historyObjectsClu.size() << std::endl;
        for (int k = 0; k < historyObjectsClu.size(); k++)
        {
            // cout<<YELLOW<<k<<" : "<<historyObjectsClu[k].objectstrajectory.size()<<endl;
            // for(int l=0;l<historyObjectsClu[k].objectstrajectory.size();l++)
            // {
            //     cout<<YELLOW<<historyObjectsClu[k].objectstrajectory[l].x<<" "<<historyObjectsClu[k].objectstrajectory[l].y<<endl;
            // }
        }
        return predictionObjects;
    }

private:
    int pridnums = 25;
    int lifeCyele = 500;
    int predictionTime = 2000;
    int newestTime = 0;

    vector<simplePredictionInfo> historyObjects;
    vector<simplePredictionInfoClu> historyObjectsClu;
    vector<cv::Point2d> Prid2(cv::Point2d x1, cv::Point2d x2)
    {
        vector<cv::Point2d> prid;
        double detlaX = x1.x - x2.x;
        double detlaY = x1.y - x2.y;
        cv::Point2d temp = x1;
        for (int i = 0; i < pridnums; i++)
        {
            double temp_V = 0.2;
            if (detlaX > temp_V)
                detlaX = temp_V;
            if (detlaX < -temp_V)
                detlaX = -temp_V;
            if (detlaY > temp_V)
                detlaY = temp_V;
            if (detlaY < -temp_V)
                detlaY = -temp_V;
            temp.x += detlaX;
            temp.y += detlaY;
            prid.push_back(temp);
        }
        return prid;
    };
    vector<cv::Point2d> Prid(simplePredictionInfo trajectory)
    {
        vector<cv::Point2d> prid;
        int size = trajectory.fata.size();
        // std::cout<<"trajectory.fata.size();"<<trajectory.fata.size()<<std::endl;
        int predictionSize = 0;
        predictionSize = ceil(size * predictionTime / trajectory.life);
        predictionSize = 20; // 固定为20个
        // std::cout<<"predictionSize;"<<predictionSize<<std::endl;
        double meanX = 0;
        double meanY = 0;
        for (int i = 0; i < size; i++)
        {
            meanX += trajectory.fata[i].lon();
            meanY += trajectory.fata[i].lat();
        }
        meanX /= size;
        meanY /= size;
        double detlaX = (trajectory.fata[0].lon() - trajectory.fata[size - 1].lon()) / size;
        double detlaY = (trajectory.fata[0].lat() - trajectory.fata[size - 1].lat()) / size;
        if (detlaX == 0 && detlaY == 0)
        {
            vector<cv::Point2d> prid;
            cv::Point2d temp;
            temp.x = trajectory.fata[0].lon();
            temp.y = trajectory.fata[0].lat();

            for (int i = 0; i < predictionSize; i++)
                prid.push_back(temp);
            return prid;
        }
        if (abs(detlaX) > abs(detlaY))
        {
            double sum1 = 0;
            double sum2 = 0;
            for (int i = 0; i < size; i++)
            {
                sum1 += trajectory.fata[i].lon() * trajectory.fata[i].lat();
                sum2 += trajectory.fata[i].lon() * trajectory.fata[i].lon();
            }
            sum1 = (sum1 - size * meanX * meanY);
            sum2 = (sum2 - size * meanX * meanX);
            double k = sum1 / sum2;
            cv::Point2d temp;
            vector<cv::Point2d> prid;
            detlaY = detlaX * k;
            temp.x = meanX + detlaX / 2 * size;
            temp.y = meanY + detlaY / 2 * size;
            for (int i = 0; i < predictionSize; i++)
            {
                temp.x += detlaX;
                temp.y += detlaY;
                if (i % 5 == 0)
                {
                    prid.push_back(temp);
                }
            }
            return prid;
        }
        else
        {
            double sum1 = 0;
            double sum2 = 0;
            for (int i = 0; i < size; i++)
            {
                sum1 += trajectory.fata[i].lon() * trajectory.fata[i].lat();
                sum2 += trajectory.fata[i].lat() * trajectory.fata[i].lat();
            }
            sum1 = (sum1 - size * meanX * meanY);
            sum2 = (sum2 - size * meanY * meanY);
            double k = sum1 / sum2;
            cv::Point2d temp;
            vector<cv::Point2d> prid;
            detlaX = detlaY * k;
            temp.x = meanX + detlaX / 2 * size;
            temp.y = meanY + detlaY / 2 * size;
            for (int i = 0; i < predictionSize; i++)
            {
                temp.x += detlaX;
                temp.y += detlaY;
                if (i % 5 == 0)
                { // 预测长度不变，减少预测个数
                    prid.push_back(temp);
                }
            }
            return prid;
        }
    }
    // 适用于激光识别聚类
    vector<cv::Point2d> Prid4(simplePredictionInfoClu trajectory)
    {
        vector<cv::Point2d> prid;
        int size = trajectory.fata.size();
        // std::cout<<"trajectory.fata.size();"<<trajectory.fata.size()<<std::endl;
        int predictionSize = 0;
        predictionSize = ceil(size * predictionTime / trajectory.life);
        predictionSize = 20; // 固定为20个
        // std::cout<<"predictionSize;"<<predictionSize<<std::endl;
        double meanX = 0;
        double meanY = 0;
        for (int i = 0; i < size; i++)
        {
            meanX += trajectory.fata[i].h();
            meanY += trajectory.fata[i].z();
        }
        meanX /= size;
        meanY /= size;
        double detlaX = (trajectory.fata[0].h() - trajectory.fata[size - 1].h()) / size;
        double detlaY = (trajectory.fata[0].z() - trajectory.fata[size - 1].z()) / size;
        if (detlaX == 0 && detlaY == 0)
        {
            vector<cv::Point2d> prid;
            cv::Point2d temp;
            temp.x = trajectory.fata[0].h();
            temp.y = trajectory.fata[0].z();

            for (int i = 0; i < predictionSize; i++)
                // prid.push_back(temp);
                if (i % 2 == 0)
                {
                    prid.push_back(temp);
                }
            return prid;
        }
        if (abs(detlaX) > abs(detlaY))
        {
            double sum1 = 0;
            double sum2 = 0;
            for (int i = 0; i < size; i++)
            {
                sum1 += trajectory.fata[i].h() * trajectory.fata[i].z();
                sum2 += trajectory.fata[i].h() * trajectory.fata[i].h();
            }
            sum1 = (sum1 - size * meanX * meanY);
            sum2 = (sum2 - size * meanX * meanX);
            double k = sum1 / sum2;
            cv::Point2d temp;
            vector<cv::Point2d> prid;
            detlaY = detlaX * k;
            temp.x = meanX + detlaX / 2 * size;
            temp.y = meanY + detlaY / 2 * size;
            for (int i = 0; i < predictionSize; i++)
            {
                temp.x += detlaX; // 位置累加
                temp.y += detlaY;
                if (i % 2 == 0)
                {
                    prid.push_back(temp);
                }
            }
            return prid;
        }
        else
        {
            double sum1 = 0;
            double sum2 = 0;
            for (int i = 0; i < size; i++)
            {
                sum1 += trajectory.fata[i].h() * trajectory.fata[i].z();
                sum2 += trajectory.fata[i].z() * trajectory.fata[i].z();
            }
            sum1 = (sum1 - size * meanX * meanY);
            sum2 = (sum2 - size * meanY * meanY);
            double k = sum1 / sum2;
            cv::Point2d temp;
            vector<cv::Point2d> prid;
            detlaX = detlaY * k;
            temp.x = meanX + detlaX / 2 * size;
            temp.y = meanY + detlaY / 2 * size;
            for (int i = 0; i < predictionSize; i++)
            {
                temp.x += detlaX;
                temp.y += detlaY;
                if (i % 2 == 0)
                { // 预测长度不变，减少预测个数
                    prid.push_back(temp);
                }
            }
            return prid;
        }
    }
};

class Jobs
{
public:
    explicit Jobs(std::vector<void *> &receivingSocketList, std::vector<void *> &sendingSocketList,
                  RoadMap &m, std::vector<GaussRoadPoint> &stopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLaneId);
    virtual ~Jobs();

    virtual void subscriber();
    virtual void publisher();
    virtual void processorLocalPlanning();
    virtual void processorGlobalPlanning();
    // add by syp
    virtual void processorGlobalPlanningBaseLocal();                                                                                                                                                                                      // 基于本地信息全局规划
    virtual void processorGlobalPlanningBaseServer();                                                                                                                                                                                     // 基于服务器信息全局规划
    void FindPointIDatRoad(GaussRoadPoint roadPoint, int32_t roadID, int32_t &laneID, int32_t &pointID);                                                                                                                                  // 根据坐标和路ID，找laneID和pointID
    void FindRoadLaneListByRoads(Astar &as);                                                                                                                                                                                              // 根据全局规划的路，找到road-lane List,包括基于当前位置和左右两侧的lane作为起点的所有可通行lane
    void FindRoadLaneByRoads(Astar &as, std::vector<std::tuple<int32_t, int32_t>> routingListPass, int originRoad, int originLane, int originPoint, int destinationRoad, int destinationLane, RoutingList::LANE_DIRECTION leftrightLane); // 根据全局规划的路，和给定位置，找到road-lane，并添加到在routingListVector中
    virtual void request();                                                                                                                                                                                                               // 用于与服务器通讯
    void Draw();                                                                                                                                                                                                                          // 画图的线程
    // end of add by syp
    //  add by ztz20230629
    void onMouse(int event, int x, int y, int flags);

    std::deque<double> history_velocity;
    std::deque<double> history_planning_speed;
    bool pause_draw_history_flag = false;
    std::deque<double> showing_velocity;
    std::deque<double> showing_planning_speed;
    // end of zdz
private:
    std::mutex mutex;

    int sendRate, recvRate, localPlanningRate, globalPlanningRate;
    std::vector<void *> rSocketList;
    std::vector<void *> sSocketList;

    RoadMap map;

    std::vector<GaussRoadPoint> stopPoints;                         // 停车点的坐标队列，不知道为啥第一点不让用
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId; // 停车点的ID值，这个只有一个，不是队列了

    std::vector<RoutingList> routingListVector; // 全局规划结果，road和lane的序列
    // std::vector<std::tuple<int32_t, int32_t>>  routingList;//road和lane的序列
    RoutingList lastRoadList; // 全局规划或者调度发送的RoadList，保存历史记录，用于判断road是否更新，向高铁新城路测设备发送信息

    pc::Imu imu;

    prediction::ObjectList prediction;
    boost::posix_time::ptime lastPacketTime4Prediction; // 接收到上一条数据的时间

    controlData::ChassisInfo chassisInfo;
    DecisionData decisionData;

    // add by syp
    // for pad UI app 交互通讯
    ui::UiPad2VData uiPad2Vdata;                     // 平板发送到车的数据
    boost::posix_time::ptime lastPacketTime4uiPad2V; // 接收到上一条数据的时间
    ui::UiV2PadData uiV2PadData;                     // 车发送到平板的数据

    // for server交互通讯
    infopack::DecisionsListProto decisionListProto; // 服务器发送到车的数据
    infopack::TrafficLight trafficLight;
    // infopack::ObjectsProto objectsProto;//车发送到服务器的数据
    infopack::DispatchProto dispatchCmd;            // 服务器消息中解析到的本车命令
    std::vector<infopack::ObjectsProto> objectsCmd; // 服务器消息中解析到的路测信息

    // for 高铁新城路侧红绿灯信息的map
    std::map<std::string, infopack::IntersectionState> spatTrafficLightMap; // 交通信号灯信息

    //20231106 为显示视觉感知障碍物测试用
    prediction::ObjectList predictionFromVision;
    boost::posix_time::ptime lastPacketTime4PredictionFromVision; // 接收到上一条数据的时间

    // predict  lry
    std::vector<infopack::ObjectsProto> objectsCmdAndPredict; // 服务器消息中解析到的路测信息+预测的障碍物
    simplePredict SPD;
    simplePredict SPD1;
    simplePredict SPD2;
    std::vector<infopack::ObjectsProto> simplePredictionCmd;    // 简单预测的路测信息
    std::vector<infopack::ObjectsProto> simplePredictionCmdClu; // 聚类预测的路测信息，合并原始和预测

    std::vector<prediction::Object> predictionClu; // 聚类proto 原始接收聚类 1
    // prediction::ObjectList predictionClulry;       // 聚类proto
    prediction::ObjectList predictionTempGlobal; // 7存放聚类高斯计算经纬转换后的数据 2
    std::vector<prediction::Object> drawForInt;  // 聚类proto   画聚类
    prediction::ObjectList predictionAndClu;     // 聚类+聚类预测+原始prediction 碰撞检测用 3
    // end of predict
    // 20231009ADD
    std::vector<Point_time> grid_planning;
    // 20231013add
    prediction::ObjectList predictioncloud; // 点云栅格
    boost::posix_time::ptime lastPacketTimeCloudPrediction; // 接收到上一条数据的时间

    enum CAR_STATUS // 用于服务器消息中的车辆状态
    {
        APPOINTMENT = 1,
        PASSENGER_ON = 2,
        FREE = 3
    };

    std::vector<GaussRoadPoint> stopPointsFromLocal;                         // 本地文件读取的停车点坐标
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointIdFromLocal; // 本地文件读取的停车点ID

    int stopTargetPathSource; // 路径控制选择，云控平板界面path1 ：1本地； path 2： 2 云端 ，3平板  -1 ，平板不知道车辆当前状态，车辆按照自己的原有路线走

    // perception 发送的红绿灯信息
    infopack::TrafficLightFromPerc trafficLightFromPerc;
    boost::posix_time::ptime lastPacketTrafficLightFromPerc; // 接收到上一条数据的时间

    // end by syp
};

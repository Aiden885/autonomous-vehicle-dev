#include "TrafficLight.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <limits.h>

using namespace std;

std::string TrafficLight::getRoadLaneKey()
{
    std::string result;
    result = std::to_string(currentRoadID_)+
    "+"+std::to_string(currentLaneID_)+
    "+"+std::to_string(nextRoadID_)+
    "+"+std::to_string(nextLaneID_);

    //std::cout<<"TrafficLight::getRoadlaneKey() = " <<result<<std::endl;

    return result;
}  

std::string TrafficLight::CreateRoadLaneKey(int currentRoadID, int currentLaneID,  int nextRoadID, int nextLaneID)
{

    std::string result;
    result = std::to_string(currentRoadID)+
    "+"+std::to_string(currentLaneID)+
    "+"+std::to_string(nextRoadID)+
    "+"+std::to_string(nextLaneID);

    //std::cout<<"TrafficLight::getRoadlaneKey() = " <<result<<std::endl;

    return result;
}

//从文件初始化map
bool TrafficLightMap::InitMapFromFile(std::string trafficLightFileName )
{
    //打开文件
    std::ifstream readFile;
	readFile.open(trafficLightFileName, std::ios::in);

    if (!readFile.is_open())
    {
        std::cout << "Open Traffice Light file Failure!" << std::endl;
        readFile.close();
        return false;
    }

    std::string strLine;//读取的一行数据
    TrafficLight trafficLightTemp;

	while (getline(readFile,strLine))
	{
			
        if(strLine.find( "路口编号") != std::string::npos)
            continue;

        if(strLine.find("地图" ) != std::string::npos)
            continue;

        if(strLine.find("右转不看灯" ) != std::string::npos)
            continue;
   

        //分割字符串
        std::cout << strLine <<std::endl;
        std::string s;
        s.append(1, ',');
        std::regex reg(s);
        std::vector<std::string> elems(std::sregex_token_iterator(strLine.begin(), strLine.end(), reg, -1),
                                   std::sregex_token_iterator());


        if(elems.size()  != 19)
            continue;              
         
        if(elems[0].empty())
            continue;
        trafficLightTemp.intersetionID_  =  elems[0];

        if(elems[4].empty())
            continue;
        trafficLightTemp.phaseID_  =  std::stoi(elems[4]);

     
        if(elems[5].empty())
            continue;
        trafficLightTemp.currentRoadID_  =  std::stoi(elems[5]);

        if(elems[6].empty())
            continue;
        trafficLightTemp.currentLaneID_  =  std::stoi(elems[6]);

        //停车点这部分很多是空的，需要注意修改表格，现在是进到这个路就停车了
        if(elems[7].empty())
            continue;
        if(elems[7].compare("end") ==0)    
            trafficLightTemp.stopPoint_  =  INT_MAX ;
        else if(elems[7].compare("待定，道路中") ==0)    
            trafficLightTemp.stopPoint_  =  0 ;
        else    
            trafficLightTemp.stopPoint_  = std::stoi(elems[7]);

          std::cout << elems[7] <<","<<elems[7].compare("end") <<","<<elems[7].compare("待定，道路中") << std::endl;   

         if(elems[8].empty())
            continue;
        trafficLightTemp.nextRoadID_  =  std::stoi(elems[8]);

         if(elems[9].empty())
            continue;
        trafficLightTemp.nextLaneID_  =  std::stoi(elems[9]);

        if(elems[10].empty())
                continue;
         
         int nTemp   =  std::stoi(elems[10]);

        //?????这一部分不对，不能作字符的直接比较，可能是最后一个字符包含回车换行之类的东西
      
       //        std::cout << elems[10] <<std::endl;
        // std::cout << elems[10] <<","<<elems[10].compare("直行") <<","<<elems[10].compare("右转")<<","<<elems[10].compare("左转")<<","<<elems[10].compare("掉头") << std::endl;
         //std::cout << elems[10] <<","<<(elems[10] == "直行") <<","<<(elems[10] == "右转") <<","<<(elems[10] == "左转") <<","<<(elems[10] == "掉头") << std::endl;
        if(nTemp==0)       
            trafficLightTemp.directionofTravel_  =  TrafficLight::STRAIGHT;
        else if(nTemp==1)
             trafficLightTemp.directionofTravel_  =  TrafficLight::RIGHT;
        else if(nTemp==2)
             trafficLightTemp.directionofTravel_  =  TrafficLight::LEFT;
         else if(nTemp==3)
            trafficLightTemp.directionofTravel_  =  TrafficLight::UTURN;
        else
            continue;


        if(elems[12].empty())
             trafficLightTemp.gaussX_  =  0.0;
        else
            trafficLightTemp.gaussX_  =  std::stod(elems[12]);

        if(elems[13].empty())
             trafficLightTemp.gaussY_  =  0.0;
        else
            trafficLightTemp.gaussY_  =  std::stod(elems[13]);

        if(elems[14].empty())
             trafficLightTemp.height_  =  0.0;
        else
            trafficLightTemp.height_  =  std::stod(elems[14]);         

         if(elems[15].empty())
             trafficLightTemp.groupSize_  =  0;
        else
            trafficLightTemp.groupSize_  =  std::stoi(elems[15]);      

        if(elems[16].empty())
             trafficLightTemp.groupIndex_  =  0;
        else
            trafficLightTemp.groupIndex_  =  std::stoi(elems[16]);      

        if(elems[17].empty())
             trafficLightTemp.lightShapeFlag_  =  0;
        else
            trafficLightTemp.lightShapeFlag_  =  std::stoi(elems[17]);    

         if(elems[18].empty())
             trafficLightTemp.lightShape_  =  "";
        else
            trafficLightTemp.lightShape_  =  elems[18];    




    
    //千辛万苦，解包成功
    mapForRoadLane_.insert(std::pair<std::string, TrafficLight>(trafficLightTemp.getRoadLaneKey(), trafficLightTemp));

    // std::cout <<  "mapForRoadLane_.insert" << trafficLightTemp.intersetionID_   << "," <<  trafficLightTemp.phaseID_ <<"," 
    //  << trafficLightTemp.currentRoadID_ << ","<<trafficLightTemp.currentLaneID_ << ","
    //   << trafficLightTemp.nextRoadID_ << ","<<trafficLightTemp.nextLaneID_ << ","<<trafficLightTemp.stopPoint_ << ","
    //   << trafficLightTemp.directionofTravel_ <<",gassX="
    //   << trafficLightTemp.gaussX_ << ","<<trafficLightTemp.gaussY_ << ","<<trafficLightTemp.height_ << ","
    //   <<trafficLightTemp.groupSize_ << ","<<trafficLightTemp.groupIndex_ << ","<<trafficLightTemp.lightShapeFlag_
    //   << ","<<trafficLightTemp.lightShape_<<std::endl;


	}

    for (std::multimap<std::string, TrafficLight>::iterator it = mapForRoadLane_.begin(); it != mapForRoadLane_.end();it++)
     {
		cout << "std::multimap<std::string, TrafficLight>::iterator"<<it->first << "," << it->second.intersetionID_ << endl;
	}

    readFile.close();
    return true;
}


//根据roadlane信息查找对象
//注意传入参数的时候，如果是换道，全局规划可能是下一条road是一样的，这样就会存在不能找到红绿灯的问题
//另外，停车点过了之后就会找下一个路口的等，所以停车务必不能超过停车线
bool  TrafficLightMap::GetTrafficLightByRoadLane(int currentRoadID, int currentLaneID,  int currentPoint, int nextRoadID, int nextLaneID ,TrafficLight & trafficeLight)
{
    //由于道路的不规范性，这里有很多判断
    //先判断当前路上是否有红绿灯

    std::multimap<string,TrafficLight>::iterator itTraffic;
    std::string key = TrafficLight::CreateRoadLaneKey( currentRoadID,  currentLaneID, currentRoadID,  currentLaneID);

    //std::cout << "GetTrafficLightByRoadLane key  1= " << key <<std::endl;

    bool bFind = false;

    if(mapForRoadLane_.count(key) != 0)//本路段中间有红绿灯
    {
        itTraffic = mapForRoadLane_.find(key);

        int minPointID = INT_MAX;
        for(int k = 0;k != mapForRoadLane_.count(key);k++,itTraffic++)//循环找最小点的,但是不能是后面的点
        {
            //cout<<itTraffic->first<<"--"<<endl;
            TrafficLight trafficeLightTemp =  (TrafficLight) itTraffic->second;
           // cout << " trafficeLightTemp.intersetionID_ " << trafficeLightTemp.intersetionID_  << " trafficeLightTemp.phaseID_"<<trafficeLightTemp.phaseID_ << std::endl;

            if(trafficeLightTemp.stopPoint_ <= currentPoint)//已经走过的点
                continue;

            if(trafficeLightTemp.stopPoint_ <= minPointID )
            {
                trafficeLight =  trafficeLightTemp;
                minPointID =  trafficeLight.stopPoint_;

                bFind = true;
            }   
        }
    }
   
   if(bFind)
        return true;    

    //如果下一条道不是当前道路，主要是针对换道的问题
    if(currentRoadID ==nextRoadID )//上面已经处理过了，如果没有红绿灯的画，就真的没有了
    {
        return false;
    }

    //查找本路段和下一路段之间的红绿灯
    key = TrafficLight::CreateRoadLaneKey( currentRoadID,  currentLaneID, nextRoadID,  nextLaneID);
   // std::cout << "GetTrafficLightByRoadLane key  2= " << key <<std::endl;

     if(mapForRoadLane_.count(key) == 0)//本路段和下一路段上没有红绿灯
    {
        //std::cout << "not find value = " <<std::endl;
        return false;
    }

    itTraffic = mapForRoadLane_.find(key);

    int minPointID = INT_MAX;
    for(int k = 0;k != mapForRoadLane_.count(key);k++,itTraffic++)//循环找最小点的
    {
        //cout<<itTraffic->first<<"--"<<endl;
        TrafficLight trafficeLightTemp =  (TrafficLight) itTraffic->second;
        //cout << " trafficeLightTemp.intersetionID_ " << trafficeLightTemp.intersetionID_  << " trafficeLightTemp.phaseID_"<<trafficeLightTemp.phaseID_ << std::endl;

        if(trafficeLightTemp.stopPoint_ <= minPointID )
        {
            trafficeLight =  trafficeLightTemp;
            minPointID =  trafficeLight.stopPoint_;
        }   
    }
		

    return true;
}
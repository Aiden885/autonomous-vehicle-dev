#include "controllerBase.hpp"
#include <cmath>
#include <iostream>
#include <cassert>
#include "printColor.h"
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "debugVisualizer.hpp"
#include "steerSafetyFilter.h"
#include "calcPreviewDistByCurvature.hpp"

#include "mathC.h"
#include "pidController.h"
#include "purePursuit.h"
#include "getPreviewIndex.h"
#include "getNearestIndex.h"
#include "calcAcceleration.h"

#include "stateDataStructTrans.h"
#include "trajDataStructTrans.h"


// #include "trajPoint.h"

namespace Control
{
    // === LatControllerBase 实现 ===
    LatControllerBase::LatControllerBase()
    {
    }

    LatControllerBase::~LatControllerBase()
    {
    }

    pureLatController::pureLatController(const YAML::Node& config)
    {
        whellBase=config["whellBase"].as<double>();
        frontWhellMaxSteer=config["frontWhellMaxSteer"].as<double>();
        dT=config["dT"].as<double>();

        previewDist = config["PreviewCTL"]["initialPreviewDistance"].as<double>();
        minPreviewDist = config["PreviewCTL"]["minPreviewDist"].as<double>();
        maxPreviewDist = config["PreviewCTL"]["maxPreviewDist"].as<double>();
        k_min=config["PreviewCTL"]["k_min"].as<double>();
        k_max=config["PreviewCTL"]["k_max"].as<double>();
        winN=config["PreviewCTL"]["winN"].as<int>();
        k_enter=config["PreviewCTL"]["k_enter"].as<double>();
        k_exit=config["PreviewCTL"]["k_exit"].as<double>();

        previewDist_lp=config["PreviewCTL"]["previewDist_lp"].as<double>();
        tau_L=config["PreviewCTL"]["tau_L"].as<double>();

        dL_downMax=config["PreviewCTL"]["dL_downMax"].as<double>();
        dL_upMax=config["PreviewCTL"]["dL_upMax"].as<double>();

        latKp = config["lat_pid"]["kp"].as<double>();
        latKd = config["lat_pid"]["ki"].as<double>();
        latKi = config["lat_pid"]["kd"].as<double>();
        int tmp = config["debugFlag"].as<int>();
        debugFlag = (tmp != 0);
    }


    double pureLatController::calcSteer(const State& s, const Traj& traj, const int& NearestIndex)
    {
        /* C结构体 */
        ::State cState;
        ::Traj  cTraj;
        /* 数据转换 */
        stateDataStructTrans(s, &cState);
        trajDataStructTrans(traj, &cTraj);


        size_t n = traj.size();
        std::vector<double> kappaList(n);
        ::smoothCurvatureSG9(&cTraj, kappaList.data());

        //@sensiz
        // std::vector<double> kappaList= smoothCurvatureSG9(traj);


        // std::cout << "beforesMK: ";
        // for (size_t i = 0; i < std::min<size_t>(20, kappaList.size()); ++i)
        // {
        //     printf("%.4f\t", traj[i].k);
        // }
        // printf("\n");
        // std::cout << "smoothedK: ";
        // for (size_t i = 0; i < std::min<size_t>(20, kappaList.size()); ++i)
        // {
        //     printf("%.4f\t", kappaList[i]);
        // }
        // printf("\n");

        double initPreviewDist=previewDist;
        int previewIdx = getPreviewIndex(&cTraj, initPreviewDist); 
        auto target = traj[previewIdx];

        const double k = kappaList[previewIdx];

        previewDist = calcPreviewDistByCurvature(
            kappaList,
            previewIdx,
            dT,

            k_min,
            k_max,
            minPreviewDist,
            maxPreviewDist,

            winN,
            k_enter,
            k_exit,

            tau_L,
            dL_upMax,
            dL_downMax,
            initPreviewDist
        );

        // previewDist=6.5;   //未限制进出弯道变化测试、进弯快、出弯慢
        previewIdx = getPreviewIndex(&cTraj, previewDist);
        target = traj[previewIdx];

        std::cout << "[InitialPreviewIdx]: index= " << previewIdx << " InitialPreviewDist: " << previewDist << std::endl;
        std::cout << "[UpdatePreviewIdx]: index= " << previewIdx << " previewDist: " << previewDist << std::endl;


        std::cout << std::fixed << std::setprecision(4);
        std::cout << "target.x: " << target.x << "  ,target.y: " << target.y <<" ,target.yaw: "<<target.yaw <<" ,target.kappa: "<<target.k<< std::endl;
        std::cout << "s.x: " << s.x << " ,s.y: " << s.y << " ,s.yaw (deg): " << s.yaw * 180.0 / M_PI << std::endl;   

        double dx=target.x-s.x;
        double dy=target.y-s.y;
        double yaw=s.yaw;   //投影到 IMU
        double prev_distance_real = sqrt(pow(dx, 2) + pow(dy, 2));
        double delta_lateral = -dx * sin(yaw) + dy * cos(yaw);

        // double pureSteer =  atan2((2.0 * delta_lateral * whellBase / pow(prev_distance_real, 2)), 1.0);

        //模块库分离测试
        ::TrajPoint cPoint;
        trajPointTrans(target, &cPoint);
        double pureSteer = purePursuit(&cState,&cPoint,whellBase);

        std::cout<<"NearestIndex: "<<NearestIndex<<std::endl;
        double dxNear=traj[NearestIndex].x-s.x;
        double dyNear=traj[NearestIndex].y-s.y;
        double NearestIdxDis2IMU = sqrt(pow(dxNear, 2) + pow(dyNear, 2));
        double NearestIdxLatDis = -dxNear * sin(yaw) + dyNear * cos(yaw);

        //@sensiz
        double error = NearestIdxLatDis;
        double kp=0.045,ki=0,kd=0.05,dt=0.05;
        double integral_max=0.3,derivative_max=3.0;
        PIDState latPid = {0};
        double feedbackSteer=pid(kp, ki, kd, dt,integral_max, derivative_max, error,&latPid);
        double steer = pureSteer + feedbackSteer;

        const double maxSteerRad = frontWhellMaxSteer * M_PI / 180.0;
        static double last_safe_rad = 0.0;
        steerSafetyFilter(steer,last_safe_rad,maxSteerRad);
 

        double pureSteerDeg=pureSteer / M_PI * 180 * 15.5;
        double feedbackSteerDeg=feedbackSteer / M_PI * 180 * 15.5;
        double finalSteerDeg= steer/ M_PI * 180 * 15.5;
        std::cout << "real prev_distance: " << prev_distance_real << " delta_x: " << dx << " delta_y: " << dy
            << "  lateral error: " << delta_lateral << std::endl;
        std::cout << "pureSteer(deg): " << pureSteerDeg  <<"  , feedbackSteer(deg): " <<  feedbackSteerDeg <<std::endl;
        std::cout << "steer(deg): " << finalSteerDeg << std::endl;


        // Control::debugVisualizer(debugFlag, s, traj, kappaList,
        //                  previewIdx, previewDist, k,
        //                  NearestIndex, NearestIdxLatDis, NearestIdxDis2IMU,
        //                  pureSteerDeg, feedbackSteerDeg, finalSteerDeg, dT);

        return steer;
    }


    // === LonController 实现 ===
    LonController::LonController() :
        vKp(0.0), vKi(0.0), vKd(0.0), aKp(0.0), aKi(0.0), aKd(0.0)
    {
    }

    LonController::LonController(const YAML::Node& config)
    {
        vKp = config["lon_pid"]["vKp"].as<double>();
        vKi = config["lon_pid"]["vKi"].as<double>();
        vKd = config["lon_pid"]["vKd"].as<double>();
        aKp = config["lon_pid"]["aKp"].as<double>();
        aKi = config["lon_pid"]["aKi"].as<double>();
        aKd = config["lon_pid"]["aKd"].as<double>();

    }

    double LonController::calcSpeed(const State& s, const Traj& traj)
    {
        if (traj.size() < 2)
        {
            std::cout << "[Warning] Trajectory too short, using first point v\n";
            return traj.front().v;
        }

        // std::cout << "[Selected] traj[1].v = " << traj[1].v << std::endl;
        return traj[1].v;
    }


    double LonController::calcAcc(const State& s, const Traj& traj)
    {

        //=================================moduleACC
        /* 速度环PID */
        double vKp = 1.0;
        double vKi = 0.1;
        double vKd = 0.01;

        /* 加速度环PID */
        double aKp = 1.5;
        double aKi = 0.2;
        double aKd = 0.02;

        /* 限幅参数 */
        double integral_max = 10.0;
        double derivative_max = 5.0;

        /* 控制周期 */
        double dT = 0.05;

        /* C结构体 */
        ::State cState;
        ::Traj  cTraj;
        /* 数据转换 */
        stateDataStructTrans(s, &cState);
        trajDataStructTrans(traj, &cTraj);

        double acc = calcAcceleration( vKp,  vKi,  vKd,
                         aKp,  aKi,  aKd,
                         integral_max,  derivative_max,
                         dT,
                         &cState, &cTraj);
        return acc;

    }

    // === Controller 实现 ===
    Controller::Controller(std::string configFile) : configFile(configFile)
    {
        YAML::Node config = YAML::LoadFile(configFile);
        ptrLatController = new pureLatController(config);
        lonController = LonController(config);
    }

    Controller::Controller()
    {
    }

    Controller::~Controller()
    {
        if (ptrLatController)
            delete ptrLatController;
    }

    ControlCMD Controller::calcControlCMD(const State& s, const Traj& trajRaw)
    {

        /* C结构体 */
        ::State cState;
        ::Traj  cTraj;
        /* 数据转换 */
        stateDataStructTrans(s, &cState);
        trajDataStructTrans(trajRaw, &cTraj);

        nearestID = getNearestIndex(&cState, &cTraj);

        ControlCMD cmd;

        std::cout<<"nearestID: "<<nearestID<<std::endl;
        double dxNear=trajRaw[nearestID].x-s.x;
        double dyNear=trajRaw[nearestID].y-s.y;
        double NearestIdxDis2IMU = sqrt(pow(dxNear, 2) + pow(dyNear, 2));

        if (NearestIdxDis2IMU >= 1.5 || trajRaw.empty()) {
            // emergency stop
            cmd = {};
            cmd.driveMode = 0;  // human drive
            std::cout<<RED<<"@@@Emergency!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            std::cout<<"The vehicle is too far from the nearest point on the planned trajectory,triggering an emergency stop and switching to manual driving mode. "<<RESET<<std::endl;

        } else 
        {
            // normal control
            cmd.steer = ptrLatController->calcSteer(s, trajRaw, nearestID);
            cmd.speed = lonController.calcSpeed(s, trajRaw);
            cmd.acceleration = lonController.calcAcc(s, trajRaw);
            cmd.driveMode = 4;
        }


        auto centerLine = [](const std::string& content, int totalWidth = 51)
        {
            int padding = totalWidth - 6 - content.length(); // 去掉两边 *
            int leftPad = padding / 2;
            int rightPad = padding - leftPad;
            std::ostringstream oss;
            oss << "***" << std::string(leftPad, ' ')
                << content << std::string(rightPad, ' ') << "***";
            return oss.str();
        };

        std::cout << BOLDBLUE;
        std::cout << "***************************************************" << std::endl;

        // 构建每一行内容
        std::ostringstream line1;
        line1 << "Speed: " << std::fixed << std::setprecision(2) << cmd.speed
            << " m/s  " << cmd.speed * 3.6 << " km/h";
        std::cout << centerLine(line1.str()) << std::endl;

        std::ostringstream line2;
        line2 << "Steer(deg): " << std::fixed << std::setprecision(2)
            << cmd.steer / M_PI * 180 * 15.5;
        std::cout << centerLine(line2.str()) << std::endl;

        std::ostringstream line3;
        line3 << "Acc: " << std::fixed << std::setprecision(2)
            << cmd.acceleration << " m/s^2";
        std::cout << centerLine(line3.str()) << std::endl;

        std::cout << "***************************************************" << std::endl;
        std::cout << RESET;


        return cmd;
    }
    
}

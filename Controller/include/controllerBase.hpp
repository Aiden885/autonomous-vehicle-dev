#pragma once
#include "controlData.hpp"
#include <string>
#include <yaml-cpp/yaml.h>


namespace Control
{
    template <typename T>
    T clamp_value(const T& val, const T& low, const T& high)
    {
        return std::max(low, std::min(val, high));
    }


    // =========================
    // 横向控制器基类
    // =========================
    class LatControllerBase
    {
    public:
        LatControllerBase();
        virtual ~LatControllerBase();
        virtual double calcSteer(const State &s, const Traj &traj,const int& nearestID) = 0;
    };


    // ===============================
    // 纯跟踪横向控制器
    // ===============================
    class pureLatController : public LatControllerBase
    {
    public:
        pureLatController(const YAML::Node &config);
        double calcSteer(const State &s, const Traj &traj,const int& nearestID) override;

    private:
        double dT;
        double whellBase;
        double frontWhellMaxSteer;
        double previewDist;
        double minPreviewDist;
        double maxPreviewDist;
        double latKp = 0.0, latKi = 0.0, latKd = 0.0;
        double lastError=0.0,integral =0.0;
        bool debugFlag;
        double k_min;
        double k_max;
        int winN;
        double k_enter;
        double k_exit;
        double previewDist_lp;
        double tau_L;
        double dL_downMax;
        double dL_upMax;
        double desired_rad;
        double last_safe_rad = 0.0;  // 上一帧“安全”的输出

    };



    // =========================
    // 纵向 PID 控制器
    // =========================
    class LonController
    {
    public:
        LonController();
        LonController(const YAML::Node &config);

        double calcSpeed(const State &s, const Traj &traj);
        double calcAcc(const State &s, const Traj &traj);

    private:
        double vKp, vKi, vKd;
        double aKp, aKi, aKd;
    };


    // =========================
    // 顶层控制器接口
    // =========================
    class Controller
    {
    public:
        Controller(std::string configFile);
        Controller();
        ~Controller();

        ControlCMD calcControlCMD(const State &s, const Traj &trajRaw);

    private:
        int nearestID;
        std::string configFile;
        LatControllerBase *ptrLatController = nullptr;
        LonController lonController;
    };
}

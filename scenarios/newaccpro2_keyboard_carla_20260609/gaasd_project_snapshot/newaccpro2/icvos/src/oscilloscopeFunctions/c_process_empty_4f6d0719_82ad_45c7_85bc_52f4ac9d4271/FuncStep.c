
#include "newaccpro2.h"

void FuncStep(int step, double step_size, double total_time, int port)
{

    const double desiredDistance = 15;  // 常量
    const double Kdist = 0.9;  // 常量
    const int enable = 1;  // 常量
    const double Kspeed = 0.8;  // 常量

    double temp952652;
    double temp952918;
    double temp953018;
    double temp953313;
    double temp953932;
    double temp954179;
    double temp954389;
    double temp954786;


    /* CARLA前车速度 */
    temp952918 = CARLAACCLeadSpeed();

    /* CARLA前车距离 */
    temp952652 = CARLAACCLeadDistance();

    /* CARLA自车速度 */
    temp953313 = CARLAACCEgoSpeed();

    temp953932 = temp952652 - desiredDistance;
    temp954179 = temp952918 - temp953313;
    temp954389 = temp953932 * Kdist;
    temp954786 = temp954179 * Kspeed;
    temp953018 = temp954389 + temp954786 + temp952918;
    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp953018, enable);

    /* 仿真示波器 */
    scope_push_send("f4e6f770-f387-459c-8fbf-c37b150ec6b9", "Component_merge_input_1", "double", temp953018, "Component_merge_input_2", "double", temp953313, "Component_merge_input_3", "double", temp952918, "Component_merge_input_4", "double", temp952652);




}
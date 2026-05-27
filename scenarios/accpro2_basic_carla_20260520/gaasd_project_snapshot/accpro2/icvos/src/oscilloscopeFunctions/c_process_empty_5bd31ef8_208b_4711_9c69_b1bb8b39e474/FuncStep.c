
#include "accpro2.h"

void FuncStep(int step, double step_size, double total_time, int port)
{
    
    const double desiredDistance = 15;  // 常量
    const double Kdist = 0.35;  // 常量
    const double Kspeed = 0.8;  // 常量
    const int enable = 1;  // 常量
    
    double temp935680;
    double temp936190;
    double temp936572;
    double temp936888;
    double temp937844;
    double temp938060;
    double temp938237;
    double temp938478;
    
    
    /* CARLA前车距离 */
    temp935680 = CARLAACCLeadDistance();

    /* CARLA前车速度 */
    temp936190 = CARLAACCLeadSpeed();

    /* CARLA自车速度 */
    temp936572 = CARLAACCEgoSpeed();

    temp937844 = temp935680 - desiredDistance;
    temp938060 = temp936190 - temp936572;
    temp938237 = temp937844 * Kdist;
    temp938478 = temp938060 * Kspeed;
    temp936888 = temp938237 + temp938478 + temp936190;
    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp936888, enable);

    /* 仿真示波器 */
    scope_push_send("76220449-3207-45db-bb17-4af989c684b6", "Component_merge_input_1", "double", temp936888, "Component_merge_input_2", "double", temp936572, "Component_merge_input_3", "double", temp936190, "Component_merge_input_4", "double", temp935680);


    
    
}
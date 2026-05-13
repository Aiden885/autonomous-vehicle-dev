
#include "carla.h"

void FuncStep(int step, double step_size, double total_time, int port)
{
    
    
    double temp787718;
    double temp788084;
    double temp788263;
    double temp788531;
    
    
    /* CARLA自车速度 */
    temp787718 = CARLAACCEgoSpeed();

    /* CARLA前车速度 */
    temp788263 = CARLAACCLeadSpeed();

    /* CARLA前车距离 */
    temp788531 = CARLAACCLeadDistance();

    /* CARLA计算目标速度 */
    temp788084 = CARLAACCComputeTargetSpeed(temp787718, temp788263, temp788531);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp788084, 1);

    /* 仿真示波器 */
    scope_push_send("7753b68e-6419-4d51-9506-320700517764", "Component_merge_input_1", "double", temp788084, "Component_merge_input_2", "double", temp787718, "Component_merge_input_3", "double", temp788263, "Component_merge_input_4", "double", temp788531);


    
    
}

#include "carla.h"

void FuncStep(int step, double step_size, double total_time, int port)
{
    
    
    double temp918311;
    double temp918674;
    double temp919087;
    double temp919325;
    
    
    /* CARLA自车速度 */
    temp918311 = CARLAACCEgoSpeed();

    /* CARLA前车速度 */
    temp919087 = CARLAACCLeadSpeed();

    /* CARLA前车距离 */
    temp919325 = CARLAACCLeadDistance();

    /* CARLA计算目标速度 */
    temp918674 = CARLAACCComputeTargetSpeed(temp355675, temp356341, temp356668);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp918674, 1);

    /* 仿真示波器 */
    scope_push_send("7753b68e-6419-4d51-9506-320700517764", "Component_merge_input_1", "double", temp918674, "Component_merge_input_2", "double", temp918311, "Component_merge_input_3", "double", temp919087, "Component_merge_input_4", "double", temp919325);


    
    
}

#include "carla.h"

void main(int argc, char *argv[])
{
    
    
    double temp355675;
    double temp355916;
    double temp356341;
    double temp356668;
    
    
    /* CARLA自车速度 */
    temp355675 = CARLAACCEgoSpeed();

    /* CARLA前车速度 */
    temp356341 = CARLAACCLeadSpeed();

    /* CARLA前车距离 */
    temp356668 = CARLAACCLeadDistance();

    /* CARLA计算目标速度 */
    temp355916 = CARLAACCComputeTargetSpeed(temp355675, temp356341, temp356668);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp355916, 1);


    
    
}
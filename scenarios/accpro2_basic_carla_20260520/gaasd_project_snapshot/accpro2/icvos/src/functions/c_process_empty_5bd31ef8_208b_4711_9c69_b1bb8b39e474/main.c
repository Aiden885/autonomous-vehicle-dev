
#include "accpro2.h"

void main(int argc, char *argv[])
{
    
    const double desiredDistance = 15;  // 常量
    const double Kdist = 0.35;  // 常量
    const double Kspeed = 0.8;  // 常量
    const int enable = 1;  // 常量
    
    double temp518863;
    double temp519178;
    double temp519482;
    double temp519682;
    double temp520365;
    double temp520617;
    double temp521294;
    double temp522078;
    
    
    /* CARLA前车距离 */
    temp518863 = CARLAACCLeadDistance();

    /* CARLA前车速度 */
    temp519178 = CARLAACCLeadSpeed();

    /* CARLA自车速度 */
    temp519482 = CARLAACCEgoSpeed();

    temp520365 = temp518863 - desiredDistance;
    temp520617 = temp519178 - temp519482;
    temp521294 = temp520365 * Kdist;
    temp522078 = temp520617 * Kspeed;
    temp519682 = temp521294 + temp522078 + temp519178;
    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp519682, enable);


    
    
}

#include "accpro1.h"
double limitSymmetrical(double value, double limit)
{
    // if (value > limit) value = limit;
    // if (value < -limit) value = -limit;
    // return value;

    double value0 = value;
    /* 第一次判断 */
    double value1;
    if (value0 > limit)
        value1 = limit;
    else
        value1 = value0;
    /* 第二次判断 */
    double value2;
    if (value1 < -limit)
        value2 = -limit;
    else
        value2 = value1;
    return value2;
}
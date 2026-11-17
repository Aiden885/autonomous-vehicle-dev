#ifndef APA_MACROS_H
#define APA_MACROS_H

/** @brief 浮点比较精度（几何运算） */
#define GEO_EPSILON 1e-6f

/** @brief 除零保护精度 */
#define DIVIDE_ZERO_EPSILON 1e-6f

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif

/** @brief 两倍圆周率 */
#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853f
#endif

/** @brief 半圆周率 */
#ifndef APA_HALF_PI
#define APA_HALF_PI 1.5707963f
#endif

#endif

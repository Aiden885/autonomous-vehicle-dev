#include "Bezier.hpp"

/**
 * @brief 在frenet坐标系中生成多条贝塞尔曲线
 * @en_name Generate multiple Bezier curves
 * @cn_name 生成多条贝塞尔曲线
 * @type module 
 *
 * @param cn_name=参数中文名, en_name=参数英文名, desc=参数说明, unit=1, type=变量类型
 * @param[IN] 形参类型 形参变量 形参中文含义
 * @param[IN] 形参类型 形参变量 形参中文含义
 *
 * @retval 返回值类型 返回值意义中文说明
 * @granularity atomic|composite 最小原子函数|可展开函数（包含小组件）
 * @tag_level1 controller 1级组件
 * @tag_level2 LatController 2级别组件
 * @formula /
 *
 * @version 1.0
 * @date 2026-01-09
 * @author liuruyu
 */
void generateBezierPathListInFrenet(const generateBezierPathListInFrenetParam &param, const generateBezierPathListInFrenetInput &input, generateBezierPathListInFrenetOutput &output)
{
    ReferenceLine referenceLine = input.referenceLine;
    PlanningPoint frenetLocationPoint = input.frenetLocationPoint;
    std::vector<PlanningTrajectory> pathList = input.pathList;

    double s = referenceLine.referenceLinePoints.back().s;

    PlanningPoint startPoint, endPoint; // frenet  坐标系下起点和终点的坐标???
    startPoint = frenetLocationPoint;   // 车辆位置在frenet坐标系下坐标

    endPoint.s = s;
    endPoint.frenetAngle = 0;

    PlanningTrajectory curve;

    for (int i = 0; i < CURVE_NUM; i++)
    {
        // 终点位置l，分别向两侧递增
        if (i % 2 == 0)
        {
            endPoint.l = -(i + 1) / 2 * CURVE_DISTANCE;
        }
        else
        {
            endPoint.l = (i + 1) / 2 * CURVE_DISTANCE;
        }
        //  std:: cout << RED << "endPoint.l: " << endPoint.l << RESET <<std::endl;
        // std::cout << RED << "endPoint.s: " << endPoint.s << RESET << std::endl;
        generateBezierPathInFrenetParam paramGBPIF{};
        generateBezierPathInFrenetInput inputGBPIF{startPoint, endPoint, curve};
        generateBezierPathInFrenetOutput outputGBPIF{curve};

        generateBezierPathInFrenet(paramGBPIF, inputGBPIF, outputGBPIF);
        curve = outputGBPIF.curve;

        // generateBezierPathInFrenet(startPoint, endPoint, curve);

        // std::cout << RED << "endPoint.l: " << curve.planningPoints.back().l << RESET <<std::endl;
        // std::cout << RED << "curve.size: " << curve.planningPoints.size() << RESET << std::endl;
        pathList.push_back(curve);
        curve.planningPoints.clear();
    }
    output.pathList = pathList;
    return;
}

// 在这个函数中，根据起点和终点，完成了一条曲线的一段的生成，
/**
 * @brief 在frenet中生成贝塞尔曲线
 * @param[IN] param 无
 * @param[IN] input 起点、终点、初始化轨迹
 * @param[OUT] output 生成轨迹
 
 * @cn_name: 生成贝塞尔曲线
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void generateBezierPathInFrenet(const generateBezierPathInFrenetParam &param, const generateBezierPathInFrenetInput &input, generateBezierPathInFrenetOutput &output)
{
    PlanningPoint startPoint = input.startPoint;
    PlanningPoint endPoint = input.endPoint;
    PlanningTrajectory curve = input.curve;

    double ds = endPoint.s - startPoint.s;
    double dl = endPoint.l - startPoint.l;
    // double distance = sqrt(pow(ds, 2) + pow(dl, 2));
    double distance = sqrt(ds * ds + dl * dl);

    PlanningPoint controlPoint[4] = {0}; // 控制点是从车辆当前坐标开始，相当于frenet坐标系做了一个平移

    controlPoint[0].s = 0.0;
    controlPoint[0].l = 0.0;

    controlPoint[0].frenetAngle = static_cast<int32_t>(360 + startPoint.frenetAngle) % 360;

    controlPoint[3].s = ds;
    controlPoint[3].l = dl;
    // std::cout << "control point 4: "<< ds <<"; " << dl << std::endl;

    controlPoint[3].frenetAngle = static_cast<int32_t>(360 + endPoint.frenetAngle) % 360;
    // std::cout << CYAN << endPoint.frenetAngle << " ; " << startPoint.frenetAngle << " ; " << controlPoint[3].frenetAngle << std::endl;

    double controlPointDistance;
    // 生成从起点到第一段终点

    controlPointDistance = CP_DISTANCE * distance / 10; // 中间2点分别与起点和终点的距离，

    controlPoint[1].s = controlPointDistance * std::cos(startPoint.frenetAngle / 180 * M_PI);
    controlPoint[1].l = controlPointDistance * std::sin(startPoint.frenetAngle / 180 * M_PI);
    // std::cout << "control point 2: " << controlPoint[1].s << "; " << controlPoint[1].l << std::endl;
    controlPoint[1].frenetAngle = controlPoint[0].frenetAngle;

    controlPoint[2].s = controlPoint[3].s - controlPointDistance * std::cos(controlPoint[3].frenetAngle / 180 * M_PI);
    controlPoint[2].l = controlPoint[3].l - controlPointDistance * std::sin(controlPoint[3].frenetAngle / 180 * M_PI);
    // std::cout << "control point 3: " << controlPoint[2].s << "; " << controlPoint[2].l << std::endl;
    controlPoint[2].frenetAngle = controlPoint[3].frenetAngle;

    // std::cout << CYAN << controlPoint[2].x + startPoint.x << " ; " << controlPoint[2].y + startPoint.y << " ; " << controlPoint[3].angle << std::endl;
    // std::cout << CYAN << -controlPointDistance * std::sin(controlPoint[3].angle / 180 * M_PI) << " h " << -controlPointDistance * std::cos(controlPoint[3].angle / 180 * M_PI) << " h " <<
    // controlPoint[3].angle << std::endl;

    std::vector<PlanningPoint> controlPointList;
    controlPointList.clear();
    for (uint32_t i = 0; i < 4; i++)
    {
        controlPointList.push_back(controlPoint[i]);
    }

    // std::cout << "qqqqqqqqqqqqqqq " <<  std::endl;

    double dt = 1.0 / (CURVE_POINT_NUM - 1); // 步长
    for (uint32_t i = 0; i < CURVE_POINT_NUM; i++)
    {
        PlanningPoint bezierPoint, resultPoint;

        pointOnCubicBezierParam paramPCB{};
        pointOnCubicBezierInput inputPCB{controlPointList, i * dt};
        pointOnCubicBezierOutput outputPCB{bezierPoint};
        pointOnCubicBezier(paramPCB, inputPCB, outputPCB);

        bezierPoint = outputPCB.result; // 在这个函数里确定贝塞尔曲线每个点的位置
        // std::cout << "wwwwwwwwwwww" << std::endl;

        resultPoint.s = bezierPoint.s + startPoint.s; // 将曲线恢复到以路点为起点的frenet坐标系
        resultPoint.l = bezierPoint.l + startPoint.l;

        if (i == 0)
        {
            resultPoint.frenetAngle = startPoint.frenetAngle;
        }
        else
        {
            resultPoint.frenetAngle = static_cast<int32_t>(360 + atan2(resultPoint.s - curve.planningPoints[i - 1].s, resultPoint.l - curve.planningPoints[i - 1].l) / M_PI * 180) % 360;
        }
        curve.planningPoints.push_back(resultPoint);
        // std::cout << CYAN << "curve.points[i].angle" << curve.points[i].angle << std::endl;
    }
    output.curve = curve;
}

/**
 * @brief 计算贝塞尔曲线上的点
 * @param[IN] param 无
 * @param[IN] input 贝塞尔曲线控制点，参数
 * @param[OUT] output 判断结果
 
 * @cn_name: 计算贝塞尔曲线上的点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointOnCubicBezier(const pointOnCubicBezierParam &param, const pointOnCubicBezierInput &input, pointOnCubicBezierOutput &output)
{
    /*
     cp在此是四個元素的陣列:
     cp[0]為起始點，或上圖中的P0
     cp[1]為第一個控制點，或上圖中的P1
     cp[2]為第二個控制點，或上圖中的P2
     cp[3]為結束點，或上圖中的P3
     t為參數值，0 <= t <= 1
     */
    std::vector<PlanningPoint> cp = input.cp;
    double t = input.t;

    double as, bs, cs;
    double al, bl, cl;
    double tSquared, tCubed;
    PlanningPoint result;

    /*計算多項式係數*/

    cs = 3.0 * (cp[1].s - cp[0].s);
    bs = 3.0 * (cp[2].s - cp[1].s) - cs;
    as = cp[3].s - cp[0].s - cs - bs;

    cl = 3.0 * (cp[1].l - cp[0].l);
    bl = 3.0 * (cp[2].l - cp[1].l) - cl;
    al = cp[3].l - cp[0].l - cl - bl;

    /*計算位於參數值t的曲線點*/

    tSquared = t * t;
    tCubed = tSquared * t;

    // std::cout << "wwwwwwwwwwww" <<std::endl;

    result.s = (as * tCubed) + (bs * tSquared) + (cs * t) + cp[0].s;
    result.l = (al * tCubed) + (bl * tSquared) + (cl * t) + cp[0].l;
    output.result = result;
    return;
}

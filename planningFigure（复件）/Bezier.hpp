

struct generateBezierPathListInFrenetParam
{
};

struct generateBezierPathListInFrenetInput
{
    ReferenceLine referenceLine;
    PlanningPoint frenetLocationPoint;
    std::vector<PlanningTrajectory> pathList;
};

struct generateBezierPathListInFrenetOutput
{
    std::vector<PlanningTrajectory> pathList;
};
  //done
void generateBezierPathListInFrenet(const generateBezierPathListInFrenetParam &param, const generateBezierPathListInFrenetInput &input, generateBezierPathListInFrenetOutput &output);

struct pointOnCubicBezierInput
{
    std::vector<PlanningPoint> cp;
    double t;
};

struct pointOnCubicBezierOutput
{
    PlanningPoint result;
};
  //done
void pointOnCubicBezier(const pointOnCubicBezierParam &param, const pointOnCubicBezierInput &input, pointOnCubicBezierOutput &output);

struct generateBezierPathInFrenetParam
{
};

struct generateBezierPathInFrenetInput
{
    PlanningPoint startPoint;
    PlanningPoint endPoint;
    PlanningTrajectory curve;
};

struct generateBezierPathInFrenetOutput
{
    PlanningTrajectory curve;
};
  //done
void generateBezierPathInFrenet(const generateBezierPathInFrenetParam &param, const generateBezierPathInFrenetInput &input, generateBezierPathInFrenetOutput &output);

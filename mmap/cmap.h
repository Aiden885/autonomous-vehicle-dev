
#ifdef __cplusplus
extern "C" {
#endif

// 道路点
typedef struct {
    double GaussX;
    double GaussY;
    double yaw;
    double curvature;
    double s;
    double speedMax;
} GaussRoadPoint;

// 车道后继ID (结构体)
typedef struct {
    int sucRoadID;
    int sucLaneID;
} LaneSuccessorId;

// 车道
typedef struct {
    int id;
    int BazierCurNUM;
    double BazierCurDIS;
    char* turnLSR;      // 对应 std::string
    
    // 使用动态数组模拟 std::vector
    LaneSuccessorId* successorId;
    int sucCount;
    int sucCapacity;

    int* leftLaneId;
    int leftCount;
    int leftCapacity;

    int* rightLaneId;
    int rightCount;
    int rightCapacity;

    GaussRoadPoint* gaussRoadPoints;
    int pointCount;
    int pointCapacity;
} Lane;

// 道路
typedef struct {
    int id;
    int isLaneChange; // C语言没有 bool，通常用 int (0/1)
    double speedMax;

    int* successorId; // 道路的后继
    int roadSucCount;
    int roadSucCapacity;

    Lane* lanes;
    int laneCount;
    int laneCapacity;
} Road;

// 路网地图 (对应 C++ 的 RoadMap 类)
typedef struct {
    double meridianLine_;
    Road* roads;
    int roadCount;
    int roadCapacity;
} RoadMap;
#ifdef __cplusplus
}
#endif

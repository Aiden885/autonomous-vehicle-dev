#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "cmap.h"


// --- 2. 辅助函数：动态数组扩容 ---
// 简单的扩容逻辑，模拟 std::vector::push_back

void checkAndExpandRoads(RoadMap* map) {
    if (map->roadCount >= map->roadCapacity) {
        // map->roadCapacity = (map->roadCapacity == 0) ? 10 : map->roadCapacity * 2;
        if (map->roadCapacity == 0) {
            map->roadCapacity = 10;
        } else {
            map->roadCapacity = map->roadCapacity * 2;
        }
        
        map->roads = (Road*)realloc(map->roads, sizeof(Road) * map->roadCapacity);
    }
}

void checkAndExpandLanes(Road* road) {
    if (road->laneCount >= road->laneCapacity) {
        // road->laneCapacity = (road->laneCapacity == 0) ? 5 : road->laneCapacity * 2;
        if (road->laneCapacity == 0) {
            road->laneCapacity = 5;
        } else {
            road->laneCapacity = road->laneCapacity * 2;
        }
        
        road->lanes = (Lane*)realloc(road->lanes, sizeof(Lane) * road->laneCapacity);
    }
}

void addInt(int** array, int* count, int* capacity, int value) {
    if (*count >= *capacity) {
        // *capacity = (*capacity == 0) ? 5 : *capacity * 2;
        if (*capacity == 0) {
            *capacity = 5;
        } else {
            *capacity = *capacity * 2;
        }
        
        *array = (int*)realloc(*array, sizeof(int) * *capacity);
    }
    (*array)[(*count)++] = value;
}

void addPoint(GaussRoadPoint** array, int* count, int* capacity, GaussRoadPoint value) {
    if (*count >= *capacity) {
        // *capacity = (*capacity == 0) ? 20 : *capacity * 2;
        if (*capacity == 0) {
            *capacity = 20;
        } else {
            *capacity = *capacity * 2;
        }
        
        *array = (GaussRoadPoint*)realloc(*array, sizeof(GaussRoadPoint) * *capacity);
    }
    (*array)[(*count)++] = value;
}

void addLaneSuc(LaneSuccessorId** array, int* count, int* capacity, LaneSuccessorId value) {
    if (*count >= *capacity) {
        // *capacity = (*capacity == 0) ? 5 : *capacity * 2;
        if (*capacity == 0) {
            *capacity = 5;
        } else {
            *capacity = *capacity * 2;
        }
        
        *array = (LaneSuccessorId*)realloc(*array, sizeof(LaneSuccessorId) * *capacity);
    }
    (*array)[(*count)++] = value;
}


// --- 3. 核心逻辑转换 (C++ -> C) ---

// C++ 中是成员函数，C 中需要把对象指针作为第一个参数传进来
void mapAnalysis(RoadMap* self, const char* path) {
    printf("mapAnalysis start...\n");

    // 1. 解析 XML 文件
    xmlDoc* doc = xmlReadFile(path, NULL, 0);
    if (doc == NULL) {
        fprintf(stderr, "Error: could not parse file %s\n", path);
        return;
    }

    // 2. 获取根节点
    xmlNode* xmlMap = xmlDocGetRootElement(doc);
    
    // 3. 解析 meridianLine 属性
    double meridianLineTemp = 120.0;
    xmlChar* prop = xmlGetProp(xmlMap, (const xmlChar*)"meridianLine");
    if (prop != NULL) {
        meridianLineTemp = atof((const char*)prop);
        xmlFree(prop); // 记得释放 libxml2 分配的内存
    }
    self->meridianLine_ = meridianLineTemp; // 注意：C结构体不要用下划线结尾，这里保持你原命名
    printf("meridianLineTemp = %f\n", meridianLineTemp);

    // 4. 遍历 road 节点
    xmlNode* xmlRoad = xmlMap->children;
    while (xmlRoad != NULL) {
        // libxml2 会把 text 节点（换行符/空格）也当成子节点，必须过滤
        if (xmlRoad->type != XML_ELEMENT_NODE) {
            xmlRoad = xmlRoad->next;
            continue;
        }

        if (xmlStrcmp(xmlRoad->name, (const xmlChar*)"road") != 0) {
            xmlRoad = xmlRoad->next;
            continue;
        }

        printf("Found a road node\n");

        // --- 初始化临时 Road 对象 ---
        Road roadTmp = {0};

        // 解析 road 属性
        prop = xmlGetProp(xmlRoad, (const xmlChar*)"id");
        if (prop != NULL) {
            roadTmp.id = atoi((const char*)prop);
            xmlFree(prop);
        }
        printf("roadID = %d\n", roadTmp.id);

        int laneChange = 1; // 默认 true
        prop = xmlGetProp(xmlRoad, (const xmlChar*)"isLaneChange");
        if (prop != NULL) {
            if (xmlStrcmp(prop, (const xmlChar*)"false") == 0) {
                laneChange = 0;
            }
            xmlFree(prop);
        }
        roadTmp.isLaneChange = laneChange;

        prop = xmlGetProp(xmlRoad, (const xmlChar*)"speedMax");
        if (prop != NULL) {
            roadTmp.speedMax = atof((const char*)prop);
            xmlFree(prop);
        }
        printf("speedMax = %f\n", roadTmp.speedMax);

        // --- 遍历 successor (road 层级) ---
        xmlNode* xmlSuccessor = xmlRoad->children;
        while (xmlSuccessor != NULL) {
            if (xmlSuccessor->type == XML_ELEMENT_NODE && 
                xmlStrcmp(xmlSuccessor->name, (const xmlChar*)"successor") == 0) {
                
                prop = xmlGetProp(xmlSuccessor, (const xmlChar*)"successor_roadId");
                if (prop != NULL) {
                    int sid = atoi((const char*)prop);
                    addInt(&roadTmp.successorId, &roadTmp.roadSucCount, &roadTmp.roadSucCapacity, sid);
                    printf("Road SuccessorID = %d\n", sid);
                    xmlFree(prop);
                }
            }
            xmlSuccessor = xmlSuccessor->next;
        }

        // --- 遍历 lanes -> lane ---
        xmlNode* xmlLanes = xmlRoad->children;
        while (xmlLanes != NULL) {
            if (xmlLanes->type == XML_ELEMENT_NODE && 
                xmlStrcmp(xmlLanes->name, (const xmlChar*)"lanes") == 0) {
                
                xmlNode* xmlLane = xmlLanes->children;
                while (xmlLane != NULL) {
                    if (xmlLane->type == XML_ELEMENT_NODE && 
                        xmlStrcmp(xmlLane->name, (const xmlChar*)"lane") == 0) {
                        
                        // --- 初始化临时 Lane 对象 ---
                        Lane laneTmp = {0};

                        // Lane 属性
                        prop = xmlGetProp(xmlLane, (const xmlChar*)"id");
                        if (prop != NULL) {
                            laneTmp.id = atoi((const char*)prop);
                            xmlFree(prop);
                        }
                        printf("laneID = %d\n", laneTmp.id);

                        int BazierCurNUMTemp;
                        double BazierCurDISTemp;
                        char* turnLSRTemp = NULL;

                        prop = xmlGetProp(xmlLane, (const xmlChar*)"BazierCurNUM");
                        if (prop != NULL) {
                            BazierCurNUMTemp = atoi((const char*)prop);
                            xmlFree(prop);
                        }
                        // 对应 C++: laneTmp.BazierCurNUM = std::max(1,1); 硬编码为 1
                        laneTmp.BazierCurNUM = 1; 

                        prop = xmlGetProp(xmlLane, (const xmlChar*)"BazierCurDIS");
                        if (prop != NULL) {
                            BazierCurDISTemp = atof((const char*)prop);
                            xmlFree(prop);
                        }
                        if (BazierCurDISTemp < 0.0) BazierCurDISTemp = 0.0;
                        laneTmp.BazierCurDIS = BazierCurDISTemp;

                        prop = xmlGetProp(xmlLane, (const xmlChar*)"turnLSR");
                        if (prop != NULL) {
                            laneTmp.turnLSR = strdup((const char*)prop); // 必须复制字符串
                            xmlFree(prop);
                        }

                        printf(" BazierCurNUMTemp=%d  BazierCurDISTemp=%f  turnLSR=%s\n", 
                               laneTmp.BazierCurNUM, laneTmp.BazierCurDIS, laneTmp.turnLSR ? laneTmp.turnLSR : "NULL");

                        // --- Lane: successor ---
                        xmlNode* xmlLaneSuc = xmlLane->children;
                        while (xmlLaneSuc != NULL) {
                            if (xmlLaneSuc->type == XML_ELEMENT_NODE && 
                                xmlStrcmp(xmlLaneSuc->name, (const xmlChar*)"successor") == 0) {
                                
                                LaneSuccessorId lsId = {-1, -1};
                                prop = xmlGetProp(xmlLaneSuc, (const xmlChar*)"successor_roadId");
                                if (prop != NULL) {
                                    lsId.sucRoadID = atoi((const char*)prop);
                                    xmlFree(prop);
                                }
                                prop = xmlGetProp(xmlLaneSuc, (const xmlChar*)"successor_laneId");
                                if (prop != NULL) {
                                    lsId.sucLaneID = atoi((const char*)prop);
                                    xmlFree(prop);
                                }
                                addLaneSuc(&laneTmp.successorId, &laneTmp.sucCount, &laneTmp.sucCapacity, lsId);
                            }
                            xmlLaneSuc = xmlLaneSuc->next;
                        }

                        // --- Lane: leftLaneID ---
                        xmlNode* xmlLeft = xmlLane->children;
                        while (xmlLeft != NULL) {
                            if (xmlLeft->type == XML_ELEMENT_NODE && 
                                xmlStrcmp(xmlLeft->name, (const xmlChar*)"leftLaneID") == 0) {
                                prop = xmlGetProp(xmlLeft, (const xmlChar*)"id");
                                if (prop != NULL) {
                                    int lid = atoi((const char*)prop);
                                    addInt(&laneTmp.leftLaneId, &laneTmp.leftCount, &laneTmp.leftCapacity, lid);
                                    xmlFree(prop);
                                }
                            }
                            xmlLeft = xmlLeft->next;
                        }

                        // --- Lane: rightLaneID ---
                        xmlNode* xmlRight = xmlLane->children;
                        while (xmlRight != NULL) {
                            if (xmlRight->type == XML_ELEMENT_NODE && 
                                xmlStrcmp(xmlRight->name, (const xmlChar*)"rightLaneID") == 0) {
                                prop = xmlGetProp(xmlRight, (const xmlChar*)"id");
                                if (prop != NULL) {
                                    int rid = atoi((const char*)prop);
                                    addInt(&laneTmp.rightLaneId, &laneTmp.rightCount, &laneTmp.rightCapacity, rid);
                                    xmlFree(prop);
                                }
                            }
                            xmlRight = xmlRight->next;
                        }

                        // --- Lane: roadPoints ---
                        xmlNode* xmlPoints = xmlLane->children;
                        while (xmlPoints != NULL) {
                            if (xmlPoints->type == XML_ELEMENT_NODE && 
                                xmlStrcmp(xmlPoints->name, (const xmlChar*)"roadPoints") == 0) {
                                
                                xmlNode* xmlPt = xmlPoints->children;
                                while (xmlPt != NULL) {
                                    if (xmlPt->type == XML_ELEMENT_NODE && 
                                        xmlStrcmp(xmlPt->name, (const xmlChar*)"roadPoint") == 0) {
                                        
                                        GaussRoadPoint pt = {0};
                                        prop = xmlGetProp(xmlPt, (const xmlChar*)"gaussX");
                                        if (prop) { pt.GaussX = atof((char*)prop); xmlFree(prop); }
                                        
                                        prop = xmlGetProp(xmlPt, (const xmlChar*)"gaussY");
                                        if (prop) { pt.GaussY = atof((char*)prop); xmlFree(prop); }

                                        prop = xmlGetProp(xmlPt, (const xmlChar*)"yaw");
                                        if (prop) { pt.yaw = atof((char*)prop); xmlFree(prop); }

                                        prop = xmlGetProp(xmlPt, (const xmlChar*)"curvature");
                                        if (prop) { pt.curvature = atof((char*)prop); xmlFree(prop); }

                                        prop = xmlGetProp(xmlPt, (const xmlChar*)"s");
                                        if (prop) { pt.s = atof((char*)prop); xmlFree(prop); }

                                        prop = xmlGetProp(xmlPt, (const xmlChar*)"speedMax");
                                        if (prop) { pt.speedMax = atof((char*)prop); xmlFree(prop); }

                                        addPoint(&laneTmp.gaussRoadPoints, &laneTmp.pointCount, &laneTmp.pointCapacity, pt);
                                    }
                                    xmlPt = xmlPt->next;
                                }
                            }
                            xmlPoints = xmlPoints->next;
                        }

                        // 将 laneTmp 加入 roadTmp
                        checkAndExpandLanes(&roadTmp);
                        roadTmp.lanes[roadTmp.laneCount++] = laneTmp; 
                        // 注意：这里直接赋值是浅拷贝，但在当前上下文中 laneTmp 是栈上的临时变量，
                        // 其指针成员（如 turnLSR）是 malloc 出来的。拷贝后 roadTmp.lanes 拿走了指针。
                        // 这在 C 中是可行的（类似移动语义），只要后续释放时小心即可。
                    }
                    xmlLane = xmlLane->next;
                }
            }
            xmlLanes = xmlLanes->next;
        }

        // 将 roadTmp 加入 self
        checkAndExpandRoads(self);
        self->roads[self->roadCount++] = roadTmp;
        
        xmlRoad = xmlRoad->next;
    }

    // 5. 清理
    xmlFreeDoc(doc);
    // xmlCleanupParser(); // 通常在程序结束时调用一次即可
}

// 升序比较函数 (用于 leftLaneId)
int compare_asc(const void* a, const void* b) {
    return (*(int*)a - *(int*)b);
}

// 降序比较函数 (用于 rightLaneId)
int compare_desc(const void* b, const void* a) {
    return (*(int*)a - *(int*)b);
}
void neighborLaneSort(RoadMap* self) {
    if (self == NULL || self->roads == NULL) return;

    printf("neighborLaneSort() roads.size() %d\n", self->roadCount);

    for (int i = 0; i < self->roadCount; i++) {
        Road* road = &self->roads[i];
        
        for (int j = 0; j < road->laneCount; j++) {
            Lane* lane = &road->lanes[j];

            // 1. 对 leftLaneId 进行升序排序 (std::sort 默认行为)
            if (lane->leftLaneId != NULL && lane->leftCount > 1) {
                qsort(lane->leftLaneId, 
                      lane->leftCount, 
                      sizeof(int), 
                      compare_asc);
            }

            // 2. 对 rightLaneId 进行降序排序 (对应 std::sort 的 rbegin/rend)
            if (lane->rightLaneId != NULL && lane->rightCount > 1) {
                qsort(lane->rightLaneId, 
                      lane->rightCount, 
                      sizeof(int), 
                      compare_desc);
            }
        }
    }
}

void moduleSelfCheck(const RoadMap* self) {
    if (self == NULL) return;

    printf("moduleSelfCheck()   {\n");
    int flag = 0;

    for (int i = 0; i < self->roadCount; i++) {
        Road* road = &self->roads[i];

        // 1. 检查 Road ID
        if (road->id < 0) {
            printf("第 %d 条道路 ID 为 %d, ID 应大于等于 0。\n", i, road->id);
            flag++;
        }

        // 2. 检查 Road 后继 ID
        for (int j = 0; j < road->roadSucCount; j++) {
            if (road->successorId[j] < 0) {
                printf("第 %d 条道路 ID 为 %d, 后继 ID 为 %d, 后继道路 ID 应大于等于 0。\n", 
                        i, road->id, road->successorId[j]);
                flag++;
            }
        }

        // 3. 检查 Lanes
        for (int j = 0; j < road->laneCount; j++) {
            Lane* lane = &road->lanes[j];

            if (lane->id < 0) {
                printf("该路段第 %d 条车道 ID 为 %d, 车道 ID 应大于等于 0。\n", j, lane->id);
                flag++;
            }

            // 检查 Lane 后继
            for (int k = 0; k < lane->sucCount; k++) {
                if (lane->successorId[k].sucRoadID < 0 || lane->successorId[k].sucLaneID < 0) {
                    printf("该路段第 %d 条车道后继信息错误: 道路 %d, 车道 %d. ID 应大于等于 0。\n", 
                            j, lane->successorId[k].sucRoadID, lane->successorId[k].sucLaneID);
                    flag++;
                }
            }

            // 检查 左相邻车道
            for (int k = 0; k < lane->leftCount; k++) {
                if (lane->leftLaneId[k] < 0) {
                    printf("该路段第 %d 条车道左相邻车道 ID 为 %d, ID 应大于等于 0。\n", j, lane->leftLaneId[k]);
                    flag++;
                }
            }

            // 检查 右相邻车道
            for (int k = 0; k < lane->rightCount; k++) {
                if (lane->rightLaneId[k] < 0) {
                    printf("该路段第 %d 条车道右相邻车道 ID 为 %d, ID 应大于等于 0。\n", j, lane->rightLaneId[k]);
                    flag++;
                }
            }

            // 检查 路径点数量
            if (lane->pointCount == 0) {
                printf("第 %d 条道路 ID 为 %d 的车道路径点数量为 0。\n", i, road->id);
                flag++;
            }
        }
    }

    if (flag == 0) {
        printf("道路解析完毕，数据读入正确。\n");
    }
}


void moduleSelfCheckPrint(const RoadMap* self) {
    if (self == NULL) {
        printf("Error: RoadMap pointer is NULL.\n");
        return;
    }

    printf("地图总共有 %d 条道路\n", self->roadCount);

    for (int i = 0; i < self->roadCount; i++) {
        Road* road = &self->roads[i];
        
        printf("道路 id：%d\n", road->id);
        printf("是否允许变道：%s\n", road->isLaneChange ? "true" : "false");

        // 打印道路后继
        for (int j = 0; j < road->roadSucCount; j++) {
            printf("  后继道路 id：%d\n", road->successorId[j]);
        }

        printf("该道路包含 %d 条车道\n", road->laneCount);

        for (int j = 0; j < road->laneCount; j++) {
            Lane* lane = &road->lanes[j];
            printf("  车道索引 %d, 车道 id：%d\n", j, lane->id);

            // 打印车道后继
            for (int k = 0; k < lane->sucCount; k++) {
                printf("    车道后继信息 -> 后继道路：%d, 后继车道：%d\n", 
                       lane->successorId[k].sucRoadID, 
                       lane->successorId[k].sucLaneID);
            }

            // 打印左相邻车道
            for (int k = 0; k < lane->leftCount; k++) {
                printf("    左侧相邻车道 id：%d\n", lane->leftLaneId[k]);
            }

            // 打印右相邻车道
            for (int k = 0; k < lane->rightCount; k++) {
                printf("    右侧相邻车道 id：%d\n", lane->rightLaneId[k]);
            }

            printf("    该车道包含路径点数量：%d\n", lane->pointCount);

            // 打印具体的路径点信息
            for (int k = 0; k < lane->pointCount; k++) {
                GaussRoadPoint* pt = &lane->gaussRoadPoints[k];
                printf("      gaussX:%.6f; gaussY:%.6f; yaw:%.6f; curvature:%.6f; s:%.6f; speedMax:%.2f\n",
                       pt->GaussX, 
                       pt->GaussY, 
                       pt->yaw, 
                       pt->curvature, 
                       pt->s, 
                       pt->speedMax);
            }
        }
    }
}
/**
 * @brief 根据 roadID 获取 road 对象
 * @param self 指向 RoadMap 对象的指针
 * @param roadID 要查找的 ID
 * @param outRoad 用于存放结果的指针（传出参数）
 * @return 找到返回 true，否则返回 false
 */
bool GetRoadByRoadID(const RoadMap* self, int roadID, Road* outRoad) {
    // 基础合法性检查
    if (self == NULL || outRoad == NULL || roadID < 0) {
        return false;
    }

    for (int i = 0; i < self->roadCount; i++) {
        if (self->roads[i].id == roadID) {
            // 在 C 语言中，结构体直接赋值是“浅拷贝”
            // 注意：如果 Road 结构体内部包含 malloc 分配的指针（如 lanes），
            // 这种赋值会导致两个结构体指向同一块动态内存。
            *outRoad = self->roads[i]; 
            return true;
        }
    }

    return false;
}


/**
 * @brief 根据 laneID 获取 lane 对象
 * @param roadPtr 指向 Road 对象的指针（输入）
 * @param laneID 要查找的车道 ID
 * @param outLane 用于存放结果的指针（输出参数）
 * @return 找到返回 true，否则返回 false
 */
bool GetLaneByLaneID(int laneID, Road road, Lane* lane) {
    if (road.id < 0 || laneID < 0 || lane == NULL)
        return false;

    for (int i = 0; i < road.laneCount; i++) {
        if (laneID == road.lanes[i].id) {
            // 结构体拷贝
            *lane = road.lanes[i];
            return true;
        }
    }
    return false;
}

// 根据 roadID 获取起点坐标
bool GetStartPointByRoadID(const RoadMap* self, int roadID, double* GaussX, double* GaussY) {
    Road road; // 局部变量，用于接收找回的道路

    // 调用第一个函数找路
    if (!GetRoadByRoadID(self, roadID, &road))
        return false;

    Lane lane; // 局部变量，用于接收找回的车道
    // 调用第二个函数找车道 (查找 ID 为 0 的车道)
    if (!GetLaneByLaneID(0, road, &lane))
        return false;

    // 检查路径点数量
    if (lane.pointCount <= 0)
        return false;

    // 赋值坐标
    *GaussX = lane.gaussRoadPoints[0].GaussX;
    *GaussY = lane.gaussRoadPoints[0].GaussY;

    return true;
}
/**
 * @brief 根据 roadID 获取该道路第一条车道的终点坐标
 * @param self 指向 RoadMap 对象的指针
 * @param roadID 目标道路 ID
 * @param GaussX 用于传出终点 X 坐标的指针
 * @param GaussY 用于传出终点 Y 坐标的指针
 * @return 成功获取返回 true，否则返回 false
 */
bool GetEndPointByRoadID(const RoadMap* self, int roadID, double* GaussX, double* GaussY) {
    Road road; // 局部变量，承接找回的道路结构体（发生值拷贝）

    // 1. 找路
    if (!GetRoadByRoadID(self, roadID, &road)) {
        return false;
    }

    Lane lane; // 局部变量，承接找回的车道结构体（发生值拷贝）
    // 2. 找车道 (按照原逻辑找 ID 为 0 的车道)
    if (!GetLaneByLaneID(0, road, &lane)) {
        return false;
    }

    // 3. 获取路径点数量
    int nPoint = lane.pointCount;
    if (nPoint <= 0) {
        return false;
    }

    // 4. 计算最后一个点的索引并提取坐标
    nPoint--; 
    *GaussX = lane.gaussRoadPoints[nPoint].GaussX;
    *GaussY = lane.gaussRoadPoints[nPoint].GaussY;

    return true;
}
/**
 * @brief 根据 roadID 和 laneID 获取 lane 对象
 * @param self 指向 RoadMap 对象的指针
 * @param roadID 目标道路 ID
 * @param LaneID 目标车道 ID
 * @param lane 用于存放结果的指针（输出参数）
 * @return 找到返回 true，否则返回 false
 */
bool GetLaneByRoadIDLaneID(const RoadMap* self, int roadID, int LaneID, Lane* lane) {
    // 基础合法性检查
    if (self == NULL || lane == NULL) {
        return false;
    }

    Road road; // 局部变量，承接找回的道路结构体（发生值拷贝）

    // 1. 找路 (调用之前定义的 GetRoadByRoadID)
    if (!GetRoadByRoadID(self, roadID, &road)) {
        return false;
    }

    // 2. 在找回的 road 中找 lane (调用之前定义的 GetLaneByLaneID)
    // 注意：你原代码中这里写的是 GetLaneByLaneID(0, ...)，
    // 如果是想找特定的 LaneID，应该把 0 换成参数 LaneID。
    // 这里先严格按照你提供的源码逻辑写为 0。
    if (!GetLaneByLaneID(0, road, lane)) {
        return false;
    }

    return true;
}
/**
 * @brief 根据 roadID, laneID, pointID 获取具体的路径点对象
 * @param self 指向 RoadMap 对象的指针
 * @param roadID 目标道路 ID
 * @param LaneID 目标车道 ID
 * @param pointID 目标点索引
 * @param guassPoint 用于存放结果点的指针（输出参数）
 * @return 找到返回 true，否则返回 false
 */
bool GetPointByRoadIDLaneID(const RoadMap* self, int roadID, int LaneID, int pointID, GaussRoadPoint* guassPoint) {
    if (self == NULL || guassPoint == NULL) {
        return false;
    }

    Road road;
    // 1. 找路
    if (!GetRoadByRoadID(self, roadID, &road)) {
        return false;
    }

    Lane lane;
    // 2. 找车道 (按照原逻辑找 ID 为 0 的车道)
    // 注意：若需寻找参数 LaneID 指定的车道，请将 0 改为 LaneID
    if (!GetLaneByLaneID(0, road, &lane)) {
        return false;
    }

    // 3. 计算最大索引
    int maxPointID = (int)lane.pointCount - 1;
    if (maxPointID < 0) { // 原逻辑是 <= 0，如果是 0 个点则返回 false
        return false;
    }

    // 4. 索引限幅 (模拟 std::min)
    // int targetIdx = (pointID < maxPointID) ? pointID : maxPointID;
    int targetIdx = 0;
    if (pointID < maxPointID) {
        targetIdx = pointID;
    } else {
        targetIdx = maxPointID;
    }
    
    if (targetIdx < 0) targetIdx = 0;

    // 5. 结构体赋值 (值拷贝)
    *guassPoint = lane.gaussRoadPoints[targetIdx];

    return true;
}
int main() {
    // TODO: 读取 XML 文件并解析
    RoadMap* map = malloc(sizeof(RoadMap));
    mapAnalysis(map, "roadMapsuzhoukcy20230315.xodr");
    neighborLaneSort(map);
    moduleSelfCheck(map);
    // moduleSelfCheckPrint(map);
    Road road;
    GetRoadByRoadID(map, 14, &road);
    printf("road id: %d\n", road.id);
    printf("road laneCount: %f\n", road.lanes[0].gaussRoadPoints[0].GaussX);
    Lane lane;
    bool islane=GetLaneByLaneID(0,road, &lane);
    printf("islane: %d\n", islane);
    printf("lane id: %d\n", lane.id);
    printf("lane pointCount: %f\n", lane.gaussRoadPoints[0].GaussX);
    double GaussX, GaussY;
    bool isID=GetStartPointByRoadID(map, 14, &GaussX, &GaussY);
    printf("isID: %d\n", isID);
    printf("GaussX: %f\n", GaussX);
    printf("GaussY: %f\n", GaussY);
    bool isID2=GetEndPointByRoadID(map, 14, &GaussX, &GaussY);
    printf("isID2: %d\n", isID2);
    printf("GaussX: %f\n", GaussX);
    printf("GaussY: %f\n", GaussY);
    bool isID3=GetLaneByRoadIDLaneID(map, 14, 0, &lane);
    printf("isID3: %d\n", isID3);
    printf("lane id: %d\n", lane.id);
    printf("lane pointCount: %f\n", lane.gaussRoadPoints[0].GaussX);
    GaussRoadPoint gaussRoadPoint;
    bool isID4=GetPointByRoadIDLaneID(map, 14, 0, 0, &gaussRoadPoint);
    printf("isID4: %d\n", isID4);
    printf("lane id: %d\n", lane.id);
    printf("lane pointCount: %f\n", gaussRoadPoint.GaussX);

    int roadIndex = -1;
    for (int i = 0; i < map->roadCount; i++) {
        if (12 == map->roads[i].id) {
            roadIndex = i;

            break;
        }
    }
    printf("roadIndex: %d\n", roadIndex);
    // printf("-----------------------------------map->roadCount; = %d\n",map->roadCount); // 获取道路数量
    // printf("-----------------------------------map->roads[0].laneCount; = %d\n",map->roads[0].laneCount); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].pointCount; = %d\n",map->roads[0].lanes[0].pointCount); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].gaussRoadPoints[1].GaussX; = %f\n",map->roads[0].lanes[0].gaussRoadPoints[1].GaussX); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].gaussRoadPoints[1].GaussY; = %f\n",map->roads[0].lanes[0].gaussRoadPoints[1].GaussY); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].gaussRoadPoints[1].yaw; = %f\n",map->roads[0].lanes[0].gaussRoadPoints[1].yaw); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].gaussRoadPoints[1].curvature; = %f\n",map->roads[0].lanes[0].gaussRoadPoints[1].curvature); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].gaussRoadPoints[1].s; = %f\n",map->roads[0].lanes[0].gaussRoadPoints[1].s); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].gaussRoadPoints[1].speedMax; = %f\n",map->roads[0].lanes[0].gaussRoadPoints[1].speedMax); // 获取道路数量
    // printf("-----------------------------------map->roads[0].lanes[0].pointCount; = %d\n",map->roads[0].successorId[0]); // 获取道路数量
    // printf("-----------------------------------map->roads[0].successorId[0]; = %d\n",map->roads[0].lanes[0].sucCount); // 获取道路数量
    // printf("-----------------------------------map->roads[0].successorId[0]; = %d\n",map->roads[0].lanes[0].successorId[0].sucRoadID); // 获取道路数量
    printf("-----------------------------------map->roads[0].successorId[0]; = %d\n",map->roads[0].lanes[0].pointCount); // 获取道路数量
    

    // printf("-----------------------------------map->laneCount; = %d\n",map->roads[0].laneCount); // 获取道路数量
    // printf("-----------------------------------map->roadCapacity; = %d\n",map->roads[0].laneCapacity); // 获取道路数量
    
    return 0;
}
#ifndef APA_VIRTUAL_RECOVERY_TYPES_H
#define APA_VIRTUAL_RECOVERY_TYPES_H

#include "vehicleCollisionCheckTypes.h"

#define APA_VIRTUAL_RECOVERY_POINT_MAX 32
#define APA_VIRTUAL_RECOVERY_PRIMITIVE_COUNT 10

/** @brief 恢复策略的只读标定，所有量在建立证书时冻结。 */
typedef struct {
  Real MaxLength;         /* 最大前缀路程，m。 */
  Real MaxInitialDepth;   /* 最大初始虚拟侵入深度，m。 */
  Real RestoredClearance; /* 恢复一侧最小净距，m。 */
  Real MaxSpeed;          /* 最大绝对目标速度，m/s。 */
  Real MaxDuration;       /* 最大前缀执行仿真时间，s。 */
} ApaVirtualRecoveryParam_t;

/**
 * @brief 保留的虚拟约束证书接口，当前入口要求 Enabled=false。
 * @en_name ApaVirtualRecoveryContext
 * @cn_name APA虚拟约束恢复上下文
 * @tag STRUCT
 */
typedef struct {
  Boolean Enabled;                        /* 保留开关，当前入口只接受 false。 */
  Real OriginX;                           /* 冻结起点世界 X，m。 */
  Real OriginY;                           /* 冻结起点世界 Y，m。 */
  Real OriginYaw;                         /* 冻结起点航向，rad。 */
  Real AnchorX;                           /* 此前正常准入的实测后轴世界 X，m。 */
  Real AnchorY;                           /* 此前正常准入的实测后轴世界 Y，m。 */
  Real AnchorYaw;                         /* 历史安全位姿航向，rad。 */
  Real MaxLength;                         /* 最大允许前缀路程及起点邻域半径，m。 */
  Real PrefixLength;                      /* 完整候选中恢复前缀的终止路程，m。 */
  Real MaxInitialDepth;                   /* 初始虚拟约束最大侵入深度，m。 */
  Real RestoredClearance;                 /* 前缀终点恢复一侧的最小几何净距，m。 */
  Real MaxSpeed;                          /* 恢复前缀绝对目标速度上限，m/s。 */
  Real MaxDuration;                       /* 前缀执行最大仿真时长，s。 */
  Integer LineCount;                      /* 冻结任务虚拟线段数量，1。 */
  Boolean Allowed[VCC_MAX_VIRTUAL_LINES]; /* 仅初始侵入的虚拟线可有限恢复。 */
  Real Side[VCC_MAX_VIRTUAL_LINES];       /* 历史安全一侧的法向符号，+1/-1。 */
  Real InitialClearance[VCC_MAX_VIRTUAL_LINES]; /* 初始有符号有限线段净距，m。 */
  Real LineX1[VCC_MAX_VIRTUAL_LINES];           /* 冻结虚拟线起点 X，m。 */
  Real LineY1[VCC_MAX_VIRTUAL_LINES];           /* 冻结虚拟线起点 Y，m。 */
  Real LineX2[VCC_MAX_VIRTUAL_LINES];           /* 冻结虚拟线终点 X，m。 */
  Real LineY2[VCC_MAX_VIRTUAL_LINES];           /* 冻结虚拟线终点 Y，m。 */
} ApaVirtualRecoveryContext_t;

/**
 * @brief 常曲率、单方向恢复前缀的定容原子接口。
 * @en_name ApaVirtualRecoveryPrefix
 * @cn_name APA虚拟约束恢复前缀
 * @tag STRUCT
 */
typedef struct {
  Boolean Valid;     /* 前缀终点及全部采样已通过正常/恢复准入。 */
  Integer Count;     /* 有效点数，1。 */
  Integer Direction; /* +1 前进，-1 倒车。 */
  Real Length;       /* 前缀绝对路程，m。 */
  Real Curvature;    /* 后轴常曲率，1/m。 */
  Real X[APA_VIRTUAL_RECOVERY_POINT_MAX];   /* 世界后轴 X，m。 */
  Real Y[APA_VIRTUAL_RECOVERY_POINT_MAX];   /* 世界后轴 Y，m。 */
  Real Yaw[APA_VIRTUAL_RECOVERY_POINT_MAX]; /* 航向，rad。 */
  Real Odo[APA_VIRTUAL_RECOVERY_POINT_MAX]; /* 当前点到下一点的有符号路程，m。 */
} ApaVirtualRecoveryPrefix_t;

#endif

/**
 * @file apaThirdBoundary.h
 * @brief APA 第三边界生成器 (Virtual Boundary) 对外开放主接口
 *
 * @date 2026-04-13
 */
#ifndef APA_THIRD_BOUNDARY_H
#define APA_THIRD_BOUNDARY_H

#include "baseTypes.h"
#include "apaSlotTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaPlannerConfig.h"
#include "apaThirdBoundaryUtils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 生成第三虚拟边界
 *
 * @param vehPos              当前车辆位置 [X, Y, Theta]
 * @param slotClass           当前目标车位类别 (EnumSlotClass)
 * @param surroundingType     周边环境类型 (EnumSurroundingType)
 * @param parkingType         泊入方式类型 (EnumParkingType)
 * @param slotList            所有检测到的车位数组指针
 * @param slotCount           检测到的车位数量
 * @param tarSlotPtr          目标车位在 slotList 中的索引(0-based)
 * @param obs                 全局碰撞盒结构信息
 * @param slotSubStatus       车位附加状态类型 (EnumSlotSubStatus)
 * @param isNormalScene       是否为正常泊车场景 ('normal' 对应 true)
 * @param useSideExtendFlag   是否使用前侧方外展检测
 * @param vehConfig           主车物理配置常数
 * @param outLines            生成的第三方限制边界集合 (输出)
 * @return Boolean            执行是否成功
 */
Boolean apaThirdBoundaryGetBoundary(const Real vehPos[3],
                                    Integer slotClass,
                                    Integer surroundingType,
                                    Integer parkingType,
                                    const ApaSlot_t *slotList,
                                    Integer slotCount,
                                    Integer tarSlotPtr,
                                    const VehicleCollisionCheckInput_t *obs,
                                    Integer slotSubStatus,
                                    Boolean isNormalScene,
                                    Boolean useSideExtendFlag,
                                    const ApaPlannerVehConfig_t *vehConfig,
                                    ApaThirdBoundaryLineSet_t *outLines);

/**
 * @brief 生成第三虚拟边界，按 MATLAB 9 参数调用补齐默认尾参。
 *
 * Defaults: slotSubStatus = END_NONE, scene_class = normal,
 * useSideExtendFlag = false.
 */
static inline Boolean apaThirdBoundaryGetBoundaryDefault(const Real vehPos[3],
                                                         Integer slotClass,
                                                         Integer surroundingType,
                                                         Integer parkingType,
                                                         const ApaSlot_t *slotList,
                                                         Integer slotCount,
                                                         Integer tarSlotPtr,
                                                         const VehicleCollisionCheckInput_t *obs,
                                                         const ApaPlannerVehConfig_t *vehConfig,
                                                         ApaThirdBoundaryLineSet_t *outLines) {
  Boolean success;

  success = apaThirdBoundaryGetBoundary(vehPos,
                                        slotClass,
                                        surroundingType,
                                        parkingType,
                                        slotList,
                                        slotCount,
                                        tarSlotPtr,
                                        obs,
                                        APA_SLOT_SUB_END_NONE,
                                        true,
                                        false,
                                        vehConfig,
                                        outLines);
  return success;
}

/**
 * @brief 生成第三虚拟边界，按 MATLAB 10 参数调用补齐默认 scene/side_extend。
 */
static inline Boolean
apaThirdBoundaryGetBoundaryWithSlotSubStatus(const Real vehPos[3],
                                             Integer slotClass,
                                             Integer surroundingType,
                                             Integer parkingType,
                                             const ApaSlot_t *slotList,
                                             Integer slotCount,
                                             Integer tarSlotPtr,
                                             const VehicleCollisionCheckInput_t *obs,
                                             Integer slotSubStatus,
                                             const ApaPlannerVehConfig_t *vehConfig,
                                             ApaThirdBoundaryLineSet_t *outLines) {
  Boolean success;

  success = apaThirdBoundaryGetBoundary(vehPos,
                                        slotClass,
                                        surroundingType,
                                        parkingType,
                                        slotList,
                                        slotCount,
                                        tarSlotPtr,
                                        obs,
                                        slotSubStatus,
                                        true,
                                        false,
                                        vehConfig,
                                        outLines);
  return success;
}

/**
 * @brief 生成第三虚拟边界，按 MATLAB 11 参数调用补齐默认 side_extend。
 */
static inline Boolean apaThirdBoundaryGetBoundaryWithScene(const Real vehPos[3],
                                                           Integer slotClass,
                                                           Integer surroundingType,
                                                           Integer parkingType,
                                                           const ApaSlot_t *slotList,
                                                           Integer slotCount,
                                                           Integer tarSlotPtr,
                                                           const VehicleCollisionCheckInput_t *obs,
                                                           Integer slotSubStatus,
                                                           Boolean isNormalScene,
                                                           const ApaPlannerVehConfig_t *vehConfig,
                                                           ApaThirdBoundaryLineSet_t *outLines) {
  Boolean success;

  success = apaThirdBoundaryGetBoundary(vehPos,
                                        slotClass,
                                        surroundingType,
                                        parkingType,
                                        slotList,
                                        slotCount,
                                        tarSlotPtr,
                                        obs,
                                        slotSubStatus,
                                        isNormalScene,
                                        false,
                                        vehConfig,
                                        outLines);
  return success;
}

#ifdef __cplusplus
}
#endif

#endif

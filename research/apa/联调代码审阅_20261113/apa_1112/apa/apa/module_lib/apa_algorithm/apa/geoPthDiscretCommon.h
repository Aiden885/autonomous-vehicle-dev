#ifndef GEO_PTH_DISCRET_COMMON_H
#define GEO_PTH_DISCRET_COMMON_H

#include "baseTypes.h"
#include "geoLnDirectDisctetTypes.h"
#include "geoTrnDirectDisctetTypes.h"

#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

/**
 * @brief 公开路径离散器之间共享的单次调用工作区
 * @en_name GeoPthDiscretCommonWork
 * @cn_name 路径离散公共工作区
 * @tag STRUCT
 */
typedef struct {
  Integer PtCount;
  Integer StepCount;
  Integer St2Idx;
  Real StepLen;
  Real LastX;
  Real LastY;
  Real LastTheta;
  Real X[PATH_PT_MAX];
  Real Y[PATH_PT_MAX];
  Real Theta[PATH_PT_MAX];
  Real Odo[PATH_PT_MAX];
} GeoPthDiscretCommonWork_t;

/**
 * @brief 路径shortcut所需的两段元数据和车辆位姿
 * @en_name GeoPthDiscretShortcutConfig
 * @cn_name 路径shortcut配置
 * @tag STRUCT
 */
typedef struct {
  Integer ShortCutSw;
  Integer SegNum;
  Integer Seg1TrnDir;
  Integer Seg2TrnDir;
  Integer Seg1GoDir;
  Integer Seg2GoDir;
  Real Seg1Len;
  Real Seg2Len;
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real ShortCutThd;
} GeoPthDiscretShortcutConfig_t;

void geoPthDiscretCommonCommitLine(const GeoLnDirectDisctetOutput_t *lineOutput,
                                   GeoPthDiscretCommonWork_t *work);

void geoPthDiscretCommonCommitTurn(const GeoTrnDirectDisctetOutput_t *turnOutput,
                                   GeoPthDiscretCommonWork_t *work);

void geoPthDiscretCommonAppendTarget(Real targetX,
                                     Real targetY,
                                     Real targetTheta,
                                     Integer tailGoDir,
                                     Real tailLength,
                                     GeoPthDiscretCommonWork_t *work);

void geoPthDiscretCommonApplyShortcut(const GeoPthDiscretShortcutConfig_t *config,
                                      GeoPthDiscretCommonWork_t *work);

#endif

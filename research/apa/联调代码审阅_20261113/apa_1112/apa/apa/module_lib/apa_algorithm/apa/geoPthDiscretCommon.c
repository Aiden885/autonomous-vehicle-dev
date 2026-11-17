#include "geoPthDiscretCommon.h"
#include "geoPthShortCutTypes.h"
#include <math.h>

/**
 * @brief 将直线原子离散结果提交到公共工作区
 * @en_name geoPthDiscretCommonCommitLine
 * @cn_name 提交直线离散结果
 * @type module
 * @param[IN] GeoLnDirectDisctetOutput_t lineOutput 直线离散完整输出
 * @param[OUT] GeoPthDiscretCommonWork_t work 路径发布工作区
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthDiscret_Ht.m + geo_PthDiscret_Vt.m + geo_PthDiscret_Vh.m
 * @version 1.0
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretCommonCommitLine(const GeoLnDirectDisctetOutput_t *lineOutput,
                                   GeoPthDiscretCommonWork_t *work) {
  Integer index0;

  work->PtCount = lineOutput->PtCount;
  work->LastX = lineOutput->LstPtX;
  work->LastY = lineOutput->LstPtY;
  work->LastTheta = lineOutput->LstPtTheta;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    work->X[index0] = lineOutput->PthPtX[index0];
    work->Y[index0] = lineOutput->PthPtY[index0];
    work->Theta[index0] = lineOutput->PthPtTht[index0];
    work->Odo[index0] = lineOutput->PthPtOdo[index0];
    index0 = index0 + 1;
  }
}

/**
 * @brief 将圆弧原子离散结果提交到公共工作区
 * @en_name geoPthDiscretCommonCommitTurn
 * @cn_name 提交圆弧离散结果
 * @type module
 * @param[IN] GeoTrnDirectDisctetOutput_t turnOutput 圆弧离散完整输出
 * @param[OUT] GeoPthDiscretCommonWork_t work 路径发布工作区
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthDiscret_Ht.m + geo_PthDiscret_Vt.m + geo_PthDiscret_Vh.m
 * @version 1.0
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretCommonCommitTurn(const GeoTrnDirectDisctetOutput_t *turnOutput,
                                   GeoPthDiscretCommonWork_t *work) {
  Integer index0;

  work->PtCount = turnOutput->PtCount;
  work->LastX = turnOutput->LstPtX;
  work->LastY = turnOutput->LstPtY;
  work->LastTheta = turnOutput->LstPtTheta;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    work->X[index0] = turnOutput->PthPtX[index0];
    work->Y[index0] = turnOutput->PthPtY[index0];
    work->Theta[index0] = turnOutput->PthPtTht[index0];
    work->Odo[index0] = turnOutput->PthPtOdo[index0];
    index0 = index0 + 1;
  }
}

/**
 * @brief 按MATLAB饱和索引语义追加目标点并写入末段Odo
 * @en_name geoPthDiscretCommonAppendTarget
 * @cn_name 追加路径目标点
 * @type module
 * @param[IN] Real targetX 目标X
 * @param[IN] Real targetY 目标Y
 * @param[IN] Real targetTheta 目标航向
 * @param[IN] Integer tailGoDir 末段行驶方向
 * @param[IN] Real tailLength 末段长度
 * @param[OUT] GeoPthDiscretCommonWork_t work 路径发布工作区
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthDiscret_Ht.m + geo_PthDiscret_Vt.m + geo_PthDiscret_Vh.m
 * @version 1.0
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretCommonAppendTarget(Real targetX,
                                     Real targetY,
                                     Real targetTheta,
                                     Integer tailGoDir,
                                     Real tailLength,
                                     GeoPthDiscretCommonWork_t *work) {
  Integer nextCount0;
  Integer targetIndex0;

  nextCount0 = work->PtCount + 1;
  if (nextCount0 > PATH_PT_MAX) {
    nextCount0 = PATH_PT_MAX;
  }
  targetIndex0 = nextCount0 - 1;
  work->X[targetIndex0] = targetX;
  work->Y[targetIndex0] = targetY;
  work->Theta[targetIndex0] = targetTheta;
  if (nextCount0 > 1) {
    Integer odoIndex0;

    odoIndex0 = nextCount0 - 2;
    work->Odo[odoIndex0] = (Real)tailGoDir * fabsf(tailLength);
  }
  work->PtCount = nextCount0;
}

static void geoPthDiscretCommonApplyShortcutRun(const GeoPthDiscretShortcutConfig_t *config,
                                                GeoPthDiscretCommonWork_t *work) {
  GeoPthShortCutInput_t shortcutInput0;
  GeoPthShortCutOutput_t shortcutOutput0;
  Integer index0;

  shortcutInput0 = (GeoPthShortCutInput_t){0};
  shortcutInput0.ShortCutSw = config->ShortCutSw;
  shortcutInput0.SegNum = config->SegNum;
  shortcutInput0.Seg1TrnDir = config->Seg1TrnDir;
  shortcutInput0.Seg2TrnDir = config->Seg2TrnDir;
  shortcutInput0.Seg1GoDir = config->Seg1GoDir;
  shortcutInput0.Seg2GoDir = config->Seg2GoDir;
  shortcutInput0.Seg1Len = config->Seg1Len;
  shortcutInput0.Seg2Len = config->Seg2Len;
  shortcutInput0.Stp2Idx = work->St2Idx;
  shortcutInput0.PtCountIn = work->PtCount;
  shortcutInput0.VehPosX = config->VehPosX;
  shortcutInput0.VehPosY = config->VehPosY;
  shortcutInput0.VehPosTheta = config->VehPosTheta;
  shortcutInput0.ShortCutThd = config->ShortCutThd;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    shortcutInput0.PthPtXIn[index0] = work->X[index0];
    shortcutInput0.PthPtYIn[index0] = work->Y[index0];
    shortcutInput0.PthPtThtIn[index0] = work->Theta[index0];
    shortcutInput0.PthPtOdoIn[index0] = work->Odo[index0];
    index0 = index0 + 1;
  }
  shortcutOutput0 = (GeoPthShortCutOutput_t){0};
  geoPthShortCut(&shortcutInput0, &shortcutOutput0);
  work->PtCount = shortcutOutput0.PtCount;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    work->X[index0] = shortcutOutput0.PthPtX[index0];
    work->Y[index0] = shortcutOutput0.PthPtY[index0];
    work->Theta[index0] = shortcutOutput0.PthPtTht[index0];
    work->Odo[index0] = shortcutOutput0.PthPtOdo[index0];
    index0 = index0 + 1;
  }
}

/**
 * @brief 调用公共shortcut原子并提交完整点列输出
 * @en_name geoPthDiscretCommonApplyShortcut
 * @cn_name 执行路径shortcut
 * @type module
 * @param[IN] GeoPthDiscretShortcutConfig_t config 两段路径和车辆配置
 * @param[OUT] GeoPthDiscretCommonWork_t work 读取并原位更新路径发布工作区
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthShortCut.m
 * @version 1.0
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretCommonApplyShortcut(const GeoPthDiscretShortcutConfig_t *config,
                                      GeoPthDiscretCommonWork_t *work) {
  geoPthDiscretCommonApplyShortcutRun(config, work);
}

#include "geoTrnDisctetLnCLnTypes.h"
#include "geoLnDirectDisctetTypes.h"
#include "geoTrnDirectDisctetTypes.h"
#include <math.h>

static void geoTrnDisctetLnCLnInitOutput(const GeoTrnDisctetLnCLnInput_t *input,
                                         GeoTrnDisctetLnCLnOutput_t *output) {
  Integer index;

  output->SuccFlg = 0;
  output->PtCount = 0;
  index = 0;
  while (index < PATH_PT_MAX) {
    output->PthPtX[index] = 0.0f;
    output->PthPtY[index] = 0.0f;
    output->PthPtTht[index] = 0.0f;
    output->PthPtOdo[index] = 0.0f;
    index = index + 1;
  }
  output->LstPtX = input->LstPtX;
  output->LstPtY = input->LstPtY;
  output->LstPtTheta = input->LstPtTheta;
}

static void geoTrnDisctetLnCLnCopyArrays(Integer count,
                                         const Real srcX[PATH_PT_MAX],
                                         const Real srcY[PATH_PT_MAX],
                                         const Real srcTht[PATH_PT_MAX],
                                         const Real srcOdo[PATH_PT_MAX],
                                         Real dstX[PATH_PT_MAX],
                                         Real dstY[PATH_PT_MAX],
                                         Real dstTht[PATH_PT_MAX],
                                         Real dstOdo[PATH_PT_MAX]) {
  Integer idx;
  (void)count;
  idx = 0;
  while (idx < PATH_PT_MAX) {
    dstX[idx] = srcX[idx];
    dstY[idx] = srcY[idx];
    dstTht[idx] = srcTht[idx];
    dstOdo[idx] = srcOdo[idx];
    idx = idx + 1;
  }
}

/**
 * @brief 直线-圆弧-直线复合段离散，对齐 MATLAB
 *        geo_TrnDisctet_LnCLn.m：当原段角度受限于最小半径时，按
 *        line/arc/line 三段重构，并依次调用 geoLnDirectDisctet 和
 *        geoTrnDirectDisctet 维护路径数组。直线段过短的前置失败复制
 *        InPt；半径或角度乘积越界的后置失败保留零初始化路径。
 * @en_name geoTrnDisctetLnCLn
 * @cn_name 直圆直复合段离散化
 * @type module
 * @param[IN] Real Crv 输入段曲率
 * @param[IN] Integer GoDir 行驶方向1前进负1后退
 * @param[IN] Integer TrnDir 转弯方向负1右转1左转
 * @param[IN] Real StepLen 离散步长
 * @param[IN] Real ArcAngle 输入段圆弧角度
 * @param[IN] Real StPtX 输入段起点X
 * @param[IN] Real StPtY 输入段起点Y
 * @param[IN] Real StPtTheta 输入段起点航向
 * @param[IN] Real EdPtX 输入段终点X
 * @param[IN] Real EdPtY 输入段终点Y
 * @param[IN] Real EdPtTheta 输入段终点航向
 * @param[IN] Integer LstStp 末段标志1最后一段
 * @param[IN] Real Rmin 最小转弯半径
 * @param[IN] Real VehPosX 车辆位姿X
 * @param[IN] Real VehPosY 车辆位姿Y
 * @param[IN] Real VehPosTheta 车辆位姿航向
 * @param[IN] Integer SltTyp 车位类型0垂直1平行
 * @param[IN] Real TarPtX 目标点X
 * @param[IN] Real TarPtY 目标点Y
 * @param[IN] Real TarPtTheta 目标点航向
 * @param[IN] Real LstPtX 上一离散点X
 * @param[IN] Real LstPtY 上一离散点Y
 * @param[IN] Real LstPtTheta 上一离散点航向
 * @param[IN] Integer InPtCount 进入时已有点数
 * @param[IN] Real[100] InPtX 进入时点X数组
 * @param[IN] Real[100] InPtY 进入时点Y数组
 * @param[IN] Real[100] InPtTht 进入时点航向数组
 * @param[IN] Real[100] InPtOdo 进入时点里程数组
 * @param[OUT] Integer SuccFlg 成功标志0失败1成功
 * @param[OUT] Integer PtCount 有效点数
 * @param[OUT] Real[100] PthPtX 离散点X数组
 * @param[OUT] Real[100] PthPtY 离散点Y数组
 * @param[OUT] Real[100] PthPtTht 离散点航向数组
 * @param[OUT] Real[100] PthPtOdo 离散点里程数组
 * @param[OUT] Real LstPtX 最后离散点X
 * @param[OUT] Real LstPtY 最后离散点Y
 * @param[OUT] Real LstPtTheta 最后离散点航向
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_TrnDisctet_LnCLn.m
 * @version 1.0
 * @date 2026-05-01
 * @author aide
 */
void geoTrnDisctetLnCLn(const GeoTrnDisctetLnCLnInput_t *input,
                        GeoTrnDisctetLnCLnOutput_t *output) {
  Real th;
  Real absTh;
  Real crv;
  Real absCrv;
  Real rmin;
  Real lnL;
  Real rd;
  Integer succFlg;
  Real goDirF;
  Integer goDir;
  Integer trnDir;
  Boolean pathValid0;

  geoTrnDisctetLnCLnInitOutput(input, output);
  th = input->ArcAngle;
  absTh = fabsf(th);
  crv = input->Crv;
  absCrv = fabsf(crv);
  rmin = input->Rmin;
  goDir = input->GoDir;
  trnDir = input->TrnDir;
  goDirF = (Real)goDir;
  pathValid0 = false;

  if (absTh > APA_PI) {
    succFlg = 1;
    lnL = 0.5f;
  } else {
    if (absCrv < 1e-6f) {
      succFlg = 0;
      lnL = 0.0f;
    } else {
      lnL = fabsf(tanf(th * 0.5f)) * (1.0f / absCrv - rmin);
      if (lnL < 0.02f) {
        succFlg = 0;
      } else {
        succFlg = 1;
      }
    }
  }

  if (succFlg == 0) {
    geoTrnDisctetLnCLnCopyArrays(input->InPtCount,
                                 input->InPtX,
                                 input->InPtY,
                                 input->InPtTht,
                                 input->InPtOdo,
                                 output->PthPtX,
                                 output->PthPtY,
                                 output->PthPtTht,
                                 output->PthPtOdo);
    output->SuccFlg = 0;
    output->PtCount = input->InPtCount;
    output->LstPtX = input->LstPtX;
    output->LstPtY = input->LstPtY;
    output->LstPtTheta = input->LstPtTheta;
  } else {

    if (absTh <= APA_PI) {
      rd = rmin;
    } else {
      Real safeCrv;
      safeCrv = crv;
      if (absCrv < 1e-6f) {
        safeCrv = 1e-6f;
      }
      rd = lnL / tanf(APA_PI - th * 0.5f) + 1.0f / safeCrv;
    }

    if (rd < rmin || (rd * th) > 25.0f) {
      output->SuccFlg = 0;
    } else {
      pathValid0 = true;
    }
  }

  if (pathValid0) {
    GeoLnDirectDisctetInput_t ln1;
    GeoLnDirectDisctetOutput_t ln1Out;
    Integer idx;

    ln1 = (GeoLnDirectDisctetInput_t){0};
    ln1.LineLen = lnL;
    ln1.GoDir = goDir;
    ln1.StepLen = input->StepLen;
    ln1.EdPtX = input->StPtX + goDirF * lnL * cosf(input->StPtTheta);
    ln1.EdPtY = input->StPtY + goDirF * lnL * sinf(input->StPtTheta);
    ln1.EdPtTheta = input->StPtTheta;
    ln1.LstPtX = input->LstPtX;
    ln1.LstPtY = input->LstPtY;
    ln1.LstPtTheta = input->LstPtTheta;
    ln1.VehPosX = input->VehPosX;
    ln1.VehPosY = input->VehPosY;
    ln1.VehPosTheta = input->VehPosTheta;
    ln1.SltTyp = input->SltTyp;
    ln1.LstStp = 0;
    ln1.TarPtX = input->TarPtX;
    ln1.TarPtY = input->TarPtY;
    ln1.TarPtTheta = input->TarPtTheta;
    ln1.InPtCount = input->InPtCount;
    idx = 0;
    while (idx < PATH_PT_MAX) {
      ln1.InPtX[idx] = input->InPtX[idx];
      ln1.InPtY[idx] = input->InPtY[idx];
      ln1.InPtTht[idx] = input->InPtTht[idx];
      ln1.InPtOdo[idx] = input->InPtOdo[idx];
      idx = idx + 1;
    }
    geoLnDirectDisctet(&ln1, &ln1Out);

    GeoTrnDirectDisctetInput_t arc;
    GeoTrnDirectDisctetOutput_t arcOut;
    Real trnDirF;
    Real arcCrv;
    Real ccX;
    Real ccY;
    Real arcLen;
    Real arcEdX;
    Real arcEdY;

    trnDirF = (Real)trnDir;

    if (rd < 1e-6f) {
      rd = 1e-6f;
    }
    arcCrv = trnDirF / rd;
    ccX = ln1Out.LstPtX + rd * cosf(ln1Out.LstPtTheta + trnDirF * APA_PI * 0.5f);
    ccY = ln1Out.LstPtY + rd * sinf(ln1Out.LstPtTheta + trnDirF * APA_PI * 0.5f);
    arcLen = fabsf(th * rd);
    arcEdX = input->EdPtX - goDirF * lnL * cosf(input->EdPtTheta);
    arcEdY = input->EdPtY - goDirF * lnL * sinf(input->EdPtTheta);

    arc = (GeoTrnDirectDisctetInput_t){0};
    arc.Crv = arcCrv;
    arc.CcX = ccX;
    arc.CcY = ccY;
    arc.ArcLen = arcLen;
    arc.GoDir = goDir;
    arc.StepLen = input->StepLen;
    arc.ArcAngle = th;
    arc.StPtX = ln1Out.LstPtX;
    arc.StPtY = ln1Out.LstPtY;
    arc.StPtTheta = ln1Out.LstPtTheta;
    arc.EdPtX = arcEdX;
    arc.EdPtY = arcEdY;
    arc.EdPtTheta = input->EdPtTheta;
    arc.LstPtX = ln1Out.LstPtX;
    arc.LstPtY = ln1Out.LstPtY;
    arc.LstPtTheta = ln1Out.LstPtTheta;
    arc.VehPosX = input->VehPosX;
    arc.VehPosY = input->VehPosY;
    arc.VehPosTheta = input->VehPosTheta;
    arc.SltTyp = input->SltTyp;
    arc.LstStp = 0;
    arc.TarPtX = input->TarPtX;
    arc.TarPtY = input->TarPtY;
    arc.TarPtTheta = input->TarPtTheta;
    arc.InPtCount = ln1Out.PtCount;
    idx = 0;
    while (idx < PATH_PT_MAX) {
      arc.InPtX[idx] = ln1Out.PthPtX[idx];
      arc.InPtY[idx] = ln1Out.PthPtY[idx];
      arc.InPtTht[idx] = ln1Out.PthPtTht[idx];
      arc.InPtOdo[idx] = ln1Out.PthPtOdo[idx];
      idx = idx + 1;
    }
    geoTrnDirectDisctet(&arc, &arcOut);

    GeoLnDirectDisctetInput_t ln3;
    GeoLnDirectDisctetOutput_t ln3Out;

    ln3 = (GeoLnDirectDisctetInput_t){0};
    ln3.LineLen = lnL;
    ln3.GoDir = goDir;
    ln3.StepLen = input->StepLen;
    ln3.EdPtX = input->EdPtX;
    ln3.EdPtY = input->EdPtY;
    ln3.EdPtTheta = input->EdPtTheta;
    ln3.LstPtX = arcOut.LstPtX;
    ln3.LstPtY = arcOut.LstPtY;
    ln3.LstPtTheta = arcOut.LstPtTheta;
    ln3.VehPosX = input->VehPosX;
    ln3.VehPosY = input->VehPosY;
    ln3.VehPosTheta = input->VehPosTheta;
    ln3.SltTyp = input->SltTyp;
    ln3.LstStp = input->LstStp;
    ln3.TarPtX = input->TarPtX;
    ln3.TarPtY = input->TarPtY;
    ln3.TarPtTheta = input->TarPtTheta;
    ln3.InPtCount = arcOut.PtCount;
    idx = 0;
    while (idx < PATH_PT_MAX) {
      ln3.InPtX[idx] = arcOut.PthPtX[idx];
      ln3.InPtY[idx] = arcOut.PthPtY[idx];
      ln3.InPtTht[idx] = arcOut.PthPtTht[idx];
      ln3.InPtOdo[idx] = arcOut.PthPtOdo[idx];
      idx = idx + 1;
    }
    geoLnDirectDisctet(&ln3, &ln3Out);

    geoTrnDisctetLnCLnCopyArrays(ln3Out.PtCount,
                                 ln3Out.PthPtX,
                                 ln3Out.PthPtY,
                                 ln3Out.PthPtTht,
                                 ln3Out.PthPtOdo,
                                 output->PthPtX,
                                 output->PthPtY,
                                 output->PthPtTht,
                                 output->PthPtOdo);
    output->SuccFlg = 1;
    output->PtCount = ln3Out.PtCount;
    output->LstPtX = ln3Out.LstPtX;
    output->LstPtY = ln3Out.LstPtY;
    output->LstPtTheta = ln3Out.LstPtTheta;
  }
}

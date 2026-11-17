#include "geoPosTypClassPkInTypes.h"
#include "geoAxisTransTypes.h"
#include <math.h>

static Boolean geoPosTypClassCrossPos(Real ptRx, Real ptRy, Real ptRtheta) {
  Real tanPr;
  Real cotPr;
  Real crossOffset;
  Real crossX;
  Boolean condThetaZero;
  Boolean condLateralZero;
  Boolean condNegativeZero;
  Boolean condCrossPos;

  condThetaZero = (ptRtheta == 0.0f);
  condLateralZero = (ptRy == 0.0f);
  condNegativeZero = signbit(ptRtheta) != 0;
  if (condThetaZero) {
    if (condLateralZero) {
      condCrossPos = false;
    } else {
      if (condNegativeZero) {
        condCrossPos = (ptRy > 0.0f);
      } else {
        condCrossPos = (ptRy < 0.0f);
      }
    }
  } else {
    tanPr = tanf(ptRtheta);
    cotPr = 1.0f / tanPr;
    crossOffset = cotPr * ptRy;
    crossX = ptRx - crossOffset;
    condCrossPos = (crossX > 0.0f);
  }
  return condCrossPos;
}

static Integer geoPosTypClassPkInLeftSlotRegion(Real ptRtheta) {
  Integer result;
  Real64 theta64 = (Real64)ptRtheta;
  Real64 negEps = -1.0e-5;
  Real64 posEps = 1.0e-5;
  Real negHalfPi = -APA_HALF_PI;
  if (theta64 <= negEps) {
    if (ptRtheta > negHalfPi) {
      result = 1;
    } else {
      result = 2;
    }
  } else {
    if (theta64 > posEps) {
      if (ptRtheta < APA_HALF_PI) {
        result = 3;
      } else {
        result = 4;
      }
    } else {
      result = 5;
    }
  }
  return result;
}

static Integer geoPosTypClassPkInLeftSlot(Real ptRtheta, Boolean condCrossPos, Boolean condIsFwd) {
  Integer result;
  Integer region = geoPosTypClassPkInLeftSlotRegion(ptRtheta);
  if (region == 5) {
    result = 17;
  } else {
    Integer baseOffset = (region - 1) * 4;
    Integer posOffset;
    if (condCrossPos) {
      posOffset = 1;
    } else {
      posOffset = 3;
    }
    Integer fwdOffset;
    if (condIsFwd) {
      fwdOffset = 0;
    } else {
      fwdOffset = 1;
    }
    result = baseOffset + posOffset + fwdOffset;
  }
  return result;
}

static Integer geoPosTypClassPkInRightSlotRegion(Real ptRtheta) {
  Integer result;
  Real negHalfPi = -APA_HALF_PI;
  if (ptRtheta >= 0.0f) {
    if (ptRtheta < APA_HALF_PI) {
      result = 1;
    } else {
      result = 2;
    }
  } else {
    if (ptRtheta > negHalfPi) {
      result = 3;
    } else {
      if (ptRtheta <= negHalfPi) {
        result = 4;
      } else {
        result = 5;
      }
    }
  }
  return result;
}

static Integer geoPosTypClassPkInRightSlot(Real ptRtheta, Boolean condCrossPos, Boolean condIsFwd) {
  Integer result;
  Integer region = geoPosTypClassPkInRightSlotRegion(ptRtheta);
  if (region == 5) {
    result = 47;
  } else {
    Integer baseOffset = 30 + (region - 1) * 4;
    Integer posOffset;
    if (condCrossPos) {
      posOffset = 1;
    } else {
      posOffset = 3;
    }
    Integer fwdOffset;
    if (condIsFwd) {
      fwdOffset = 0;
    } else {
      fwdOffset = 1;
    }
    result = baseOffset + posOffset + fwdOffset;
  }
  return result;
}

static Integer geoPosTypClassGenericLeftSlotRegion(Real ptRtheta) {
  Integer result;
  Real64 theta64;
  Real64 negEps;
  Real64 posEps;
  Real negHalfPi;

  theta64 = (Real64)ptRtheta;
  negEps = -1.0e-3;
  posEps = 1.0e-3;
  negHalfPi = -APA_HALF_PI;
  if (theta64 <= negEps) {
    if (ptRtheta > negHalfPi) {
      result = 1;
    } else {
      result = 2;
    }
  } else {
    if (theta64 > posEps) {
      if (ptRtheta < APA_HALF_PI) {
        result = 3;
      } else {
        result = 4;
      }
    } else {
      if (ptRtheta >= APA_HALF_PI) {
        result = 4;
      } else {
        result = 5;
      }
    }
  }
  return result;
}

static Integer
geoPosTypClassGenericLeftSlot(Real ptRtheta, Boolean condCrossPos, Boolean condIsFwd) {
  Integer result;
  Integer region;
  Integer baseOffset;
  Integer posOffset;
  Integer fwdOffset;

  region = geoPosTypClassGenericLeftSlotRegion(ptRtheta);
  if (region == 5) {
    result = 17;
  } else {
    baseOffset = (region - 1) * 4;
    if (condCrossPos) {
      posOffset = 1;
    } else {
      posOffset = 3;
    }
    if (condIsFwd) {
      fwdOffset = 0;
    } else {
      fwdOffset = 1;
    }
    result = baseOffset + posOffset + fwdOffset;
  }
  return result;
}

/**
 * @brief 根据起点在目标坐标系中的相对位置和行驶方向，将泊入场景分类为47种位置类型编号
 * @en_name geoPosTypClassPkIn
 * @cn_name 泊入位置分类
 * @type module
 * @param[IN] Real StPtX 起点X
 * @param[IN] Real StPtY 起点Y
 * @param[IN] Real StPtTheta 起点航向角
 * @param[IN] Real EdPtX 目标点X
 * @param[IN] Real EdPtY 目标点Y
 * @param[IN] Real EdPtTheta 目标点航向角
 * @param[IN] Integer Dir 行驶方向1前进负1后退
 * @param[OUT] Integer TarPTyp 位置类型编号1到47
 * @param[OUT] Real PtRelaX 相对坐标X
 * @param[OUT] Real PtRelaY 相对坐标Y
 * @param[OUT] Real PtRelaTheta 相对航向角
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_PosTypClassPkIn.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoPosTypClassPkIn(const GeoPosTypClassPkInInput_t *input,
                        GeoPosTypClassPkInOutput_t *output) {

  Real stX;
  Real stY;
  Real stTheta;
  Real edX;
  Real edY;
  Real edTheta;
  Integer dir;

  stX = input->StPtX;
  stY = input->StPtY;
  stTheta = input->StPtTheta;
  edX = input->EdPtX;
  edY = input->EdPtY;
  edTheta = input->EdPtTheta;
  dir = input->Dir;

  Real ptRx;
  Real ptRy;
  Real ptRtheta;
  geoAxisTransToTarget(stX, stY, stTheta, edX, edY, edTheta, &ptRx, &ptRy, &ptRtheta);

  Integer tarPTyp;
  Boolean condIsFwd;
  condIsFwd = (dir == 1);

  Boolean condLeftSlot;
  condLeftSlot = (ptRy >= 0.0f);

  Boolean condCrossPos;
  condCrossPos = geoPosTypClassCrossPos(ptRx, ptRy, ptRtheta);

  if (condLeftSlot) {
    tarPTyp = geoPosTypClassPkInLeftSlot(ptRtheta, condCrossPos, condIsFwd);
  } else {
    tarPTyp = geoPosTypClassPkInRightSlot(ptRtheta, condCrossPos, condIsFwd);
  }

  output->TarPTyp = tarPTyp;
  output->PtRelaX = ptRx;
  output->PtRelaY = ptRy;
  output->PtRelaTheta = ptRtheta;
}

/**
 * @brief 通用位置分类，对齐 geo_PosTypClass.m 的 PtRelaX 侧向判定与 1e-3 航向阈值
 * @en_name geoPosTypClass
 * @cn_name 通用位置分类
 * @type module
 * @param[IN] Real StPtX 起点X
 * @param[IN] Real StPtY 起点Y
 * @param[IN] Real StPtTheta 起点航向角
 * @param[IN] Real EdPtX 目标点X
 * @param[IN] Real EdPtY 目标点Y
 * @param[IN] Real EdPtTheta 目标点航向角
 * @param[IN] Integer Dir 行驶方向1前进负1后退
 * @param[OUT] Integer TarPTyp 位置类型编号1到47
 * @param[OUT] Real PtRelaX 相对坐标X
 * @param[OUT] Real PtRelaY 相对坐标Y
 * @param[OUT] Real PtRelaTheta 相对航向角
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_PosTypClass.m
 * @version 1.0
 * @date 2026-05-04
 * @author aide
 */
void geoPosTypClass(const GeoPosTypClassInput_t *input, GeoPosTypClassOutput_t *output) {
  Real stX;
  Real stY;
  Real stTheta;
  Real edX;
  Real edY;
  Real edTheta;
  Integer dir;
  Real ptRx;
  Real ptRy;
  Real ptRtheta;
  Integer tarPTyp;
  Boolean condIsFwd;
  Boolean condLeftSlot;
  Boolean condCrossPos;

  stX = input->StPtX;
  stY = input->StPtY;
  stTheta = input->StPtTheta;
  edX = input->EdPtX;
  edY = input->EdPtY;
  edTheta = input->EdPtTheta;
  dir = input->Dir;

  geoAxisTransToTarget(stX, stY, stTheta, edX, edY, edTheta, &ptRx, &ptRy, &ptRtheta);
  condIsFwd = (dir == 1);
  condLeftSlot = (ptRx >= 0.0f);
  condCrossPos = geoPosTypClassCrossPos(ptRx, ptRy, ptRtheta);
  if (condLeftSlot) {
    tarPTyp = geoPosTypClassGenericLeftSlot(ptRtheta, condCrossPos, condIsFwd);
  } else {
    tarPTyp = geoPosTypClassPkInRightSlot(ptRtheta, condCrossPos, condIsFwd);
  }

  output->TarPTyp = tarPTyp;
  output->PtRelaX = ptRx;
  output->PtRelaY = ptRy;
  output->PtRelaTheta = ptRtheta;
}

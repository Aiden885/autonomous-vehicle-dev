#include "apaMathUtils.h"
#include "geoVtDangerPtTypes.h"
#include "geoVtEntrPtDngPtTypes.h"
#include <math.h>

#ifndef GEO_VT_ENTR_EPS
#define GEO_VT_ENTR_EPS 1.0e-6f
#endif

#ifndef GEO_VT_ENTR_LAT_OFST_REAL
#define GEO_VT_ENTR_LAT_OFST_REAL 0.35f
#endif

#ifndef GEO_VT_ENTR_LAT_OFST_VIRTUAL
#define GEO_VT_ENTR_LAT_OFST_VIRTUAL 0.1f
#endif

#ifndef GEO_VT_ENTR_VEH_WD
#define GEO_VT_ENTR_VEH_WD 1.9f
#endif

#ifndef GEO_VT_ENTR_LAT_DANGER_THD
#define GEO_VT_ENTR_LAT_DANGER_THD 2.5f
#endif

#ifndef GEO_VT_ENTR_LEN_SHORT
#define GEO_VT_ENTR_LEN_SHORT 1.3f
#endif

#ifndef GEO_VT_ENTR_LEN_MID
#define GEO_VT_ENTR_LEN_MID 1.8f
#endif

#ifndef GEO_VT_ENTR_LEN_LONG
#define GEO_VT_ENTR_LEN_LONG 3.0f
#endif

#ifndef GEO_VT_ENTR_SURD_NONE
#define GEO_VT_ENTR_SURD_NONE 3
#endif

#ifndef GEO_VT_ENTR_SURD_LEFT_FRONT_ONLY
#define GEO_VT_ENTR_SURD_LEFT_FRONT_ONLY 2
#endif

#ifndef GEO_VT_ENTR_SURD_RIGHT_REAR_ONLY
#define GEO_VT_ENTR_SURD_RIGHT_REAR_ONLY 1
#endif

#ifndef GEO_VT_ENTR_SURD_BOTH
#define GEO_VT_ENTR_SURD_BOTH 0
#endif

#ifndef GEO_VT_ENTR_PRK_LEFT
#define GEO_VT_ENTR_PRK_LEFT 1
#endif

#ifndef GEO_VT_ENTR_PRK_RIGHT
#define GEO_VT_ENTR_PRK_RIGHT 2
#endif

typedef struct {
  Real EntrP1X;
  Real EntrP1Y;
  Real EntrP2X;
  Real EntrP2Y;
  Real LeftPtx;
  Real LeftPty;
  Real RightPtx;
  Real RightPty;
  Real LeftWidth;
  Real RightWidth;
  Real LeftFront;
  Real RightFront;
  Real CrossX;
  Real CrossY;
  Real SltDepth;
} GeoVtEntrPtDngPtGeometry_t;

typedef struct {
  Real RealX;
  Real RealY;
  Real VirtualX;
  Real VirtualY;
} GeoVtEntrPtDngPtDangerRel_t;

static Real geoVtEntrPtDngPtSafeSqrt(Real value) {
  Real input0;
  Real result0;

  if (value < 0.0f) {
    input0 = 0.0f;
  } else {
    input0 = value;
  }
  result0 = sqrtf(input0);

  return result0;
}

static Real geoVtEntrPtDngPtLimitFactor(Real lateral) {
  Real absLat0;
  Real denom0;
  Real raw0;
  Real clamped0;

  absLat0 = fabsf(lateral);
  if (absLat0 < GEO_VT_ENTR_EPS) {
    denom0 = GEO_VT_ENTR_EPS;
  } else {
    denom0 = absLat0;
  }
  raw0 = 1.0f / denom0;
  clamped0 = apaMathMax(0.4f, raw0);
  clamped0 = apaMathMin(1.0f, clamped0);

  return clamped0;
}

static void geoVtEntrPtDngPtCalcEntrLine(const GeoVtEntrPtDngPtInput_t *input,
                                         GeoVtEntrPtDngPtGeometry_t *geom) {
  Real p1x0;
  Real p1y0;
  Real p2x0;
  Real p2y0;
  Real p3x0;
  Real p3y0;
  Real p4x0;
  Real p4y0;
  Real edgeDx0;
  Real edgeDy0;
  Real edgeLenSq0;

  p1x0 = input->SltPt1X;
  p1y0 = input->SltPt1Y;
  p2x0 = input->SltPt2X;
  p2y0 = input->SltPt2Y;
  p3x0 = input->SltPt3X;
  p3y0 = input->SltPt3Y;
  p4x0 = input->SltPt4X;
  p4y0 = input->SltPt4Y;

  if (input->SltEntr == 2) {
    geom->EntrP1X = p2x0;
    geom->EntrP1Y = p2y0;
    geom->EntrP2X = p3x0;
    geom->EntrP2Y = p3y0;
  } else {
    if (input->SltEntr == 3) {
      geom->EntrP1X = p3x0;
      geom->EntrP1Y = p3y0;
      geom->EntrP2X = p4x0;
      geom->EntrP2Y = p4y0;
    } else {
      if (input->SltEntr == 4) {
        geom->EntrP1X = p4x0;
        geom->EntrP1Y = p4y0;
        geom->EntrP2X = p1x0;
        geom->EntrP2Y = p1y0;
      } else {
        geom->EntrP1X = p1x0;
        geom->EntrP1Y = p1y0;
        geom->EntrP2X = p2x0;
        geom->EntrP2Y = p2y0;
      }
    }
  }

  edgeDx0 = geom->EntrP2X - geom->EntrP1X;
  edgeDy0 = geom->EntrP2Y - geom->EntrP1Y;
  edgeLenSq0 = (edgeDx0 * edgeDx0) + (edgeDy0 * edgeDy0);
  if (edgeLenSq0 < GEO_VT_ENTR_EPS) {
    geom->EntrP1X = p1x0;
    geom->EntrP1Y = p1y0;
    geom->EntrP2X = p2x0;
    geom->EntrP2Y = p2y0;
  }
}

static void geoVtEntrPtDngPtCalcGeometry(const GeoVtEntrPtDngPtInput_t *input,
                                         GeoVtEntrPtDngPtGeometry_t *geom) {
  Real tgtX0;
  Real tgtY0;
  Real cosT0;
  Real sinT0;
  Real d1x0;
  Real d1y0;
  Real d2x0;
  Real d2y0;
  Real rel1x0;
  Real rel1y0;
  Real rel2x0;
  Real rel2y0;
  Real headVecX0;
  Real headVecY0;
  Real edgeVecX0;
  Real edgeVecY0;
  Real det0;
  Real crossX0;
  Real crossY0;
  Real numT0;
  Real ratio0;
  Real dxDepth0;
  Real dyDepth0;
  Real depthSq0;

  geoVtEntrPtDngPtCalcEntrLine(input, geom);

  tgtX0 = input->TgtX;
  tgtY0 = input->TgtY;
  cosT0 = cosf(input->TgtTheta);
  sinT0 = sinf(input->TgtTheta);

  d1x0 = geom->EntrP1X - tgtX0;
  d1y0 = geom->EntrP1Y - tgtY0;
  d2x0 = geom->EntrP2X - tgtX0;
  d2y0 = geom->EntrP2Y - tgtY0;
  rel1x0 = (cosT0 * d1x0) + (sinT0 * d1y0);
  rel1y0 = (cosT0 * d1y0) - (sinT0 * d1x0);
  rel2x0 = (cosT0 * d2x0) + (sinT0 * d2y0);
  rel2y0 = (cosT0 * d2y0) - (sinT0 * d2x0);

  if (rel1y0 > 0.0f) {
    geom->LeftWidth = rel1y0;
    geom->LeftFront = rel1x0;
    geom->RightWidth = -rel2y0;
    geom->RightFront = rel2x0;
    geom->LeftPtx = geom->EntrP1X;
    geom->LeftPty = geom->EntrP1Y;
    geom->RightPtx = geom->EntrP2X;
    geom->RightPty = geom->EntrP2Y;
  } else {
    geom->RightWidth = -rel1y0;
    geom->RightFront = rel1x0;
    geom->LeftWidth = rel2y0;
    geom->LeftFront = rel2x0;
    geom->LeftPtx = geom->EntrP2X;
    geom->LeftPty = geom->EntrP2Y;
    geom->RightPtx = geom->EntrP1X;
    geom->RightPty = geom->EntrP1Y;
  }

  headVecX0 = cosT0;
  headVecY0 = sinT0;
  edgeVecX0 = geom->RightPtx - geom->LeftPtx;
  edgeVecY0 = geom->RightPty - geom->LeftPty;
  det0 = (edgeVecX0 * headVecY0) - (edgeVecY0 * headVecX0);
  if (fabsf(det0) > GEO_VT_ENTR_EPS) {
    crossX0 = tgtX0 - geom->LeftPtx;
    crossY0 = tgtY0 - geom->LeftPty;
    numT0 = (crossX0 * headVecY0) - (crossY0 * headVecX0);
    ratio0 = numT0 / det0;
    geom->CrossX = geom->LeftPtx + (ratio0 * edgeVecX0);
    geom->CrossY = geom->LeftPty + (ratio0 * edgeVecY0);
  } else {
    geom->CrossX = (geom->LeftPtx + geom->RightPtx) * 0.5f;
    geom->CrossY = (geom->LeftPty + geom->RightPty) * 0.5f;
  }

  dxDepth0 = tgtX0 - geom->CrossX;
  dyDepth0 = tgtY0 - geom->CrossY;
  depthSq0 = (dxDepth0 * dxDepth0) + (dyDepth0 * dyDepth0);
  geom->SltDepth = sqrtf(depthSq0);
}

static void geoVtEntrPtDngPtCallDanger(const GeoVtEntrPtDngPtInput_t *input,
                                       const GeoVtEntrPtDngPtGeometry_t *geom,
                                       GeoVtDangerPtOutput_t *dangerOutput) {
  GeoVtDangerPtInput_t dangerInput;
  Integer idx0;

  dangerInput = (GeoVtDangerPtInput_t){0};
  dangerInput.SltEntrPt1X = geom->LeftPtx;
  dangerInput.SltEntrPt1Y = geom->LeftPty;
  dangerInput.SltEntrPt2X = geom->RightPtx;
  dangerInput.SltEntrPt2Y = geom->RightPty;
  dangerInput.TgtX = input->TgtX;
  dangerInput.TgtY = input->TgtY;
  dangerInput.TgtTheta = input->TgtTheta;
  dangerInput.Radius = input->Radius;
  dangerInput.PrkType = input->PrkType;
  dangerInput.BoundaryLineNum = input->BoundaryLineNum;
  dangerInput.VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
  dangerInput.BoundarySepPtNum = input->BoundarySepPtNum;
  dangerInput.StopperPtNum = input->StopperPtNum;

  idx0 = 0;
  while (idx0 < DGR_LINE_MAX) {
    dangerInput.BoundaryP1X[idx0] = input->BoundaryP1X[idx0];
    dangerInput.BoundaryP1Y[idx0] = input->BoundaryP1Y[idx0];
    dangerInput.BoundaryP2X[idx0] = input->BoundaryP2X[idx0];
    dangerInput.BoundaryP2Y[idx0] = input->BoundaryP2Y[idx0];
    dangerInput.VirtualBoundaryP1X[idx0] = input->VirtualBoundaryP1X[idx0];
    dangerInput.VirtualBoundaryP1Y[idx0] = input->VirtualBoundaryP1Y[idx0];
    dangerInput.VirtualBoundaryP2X[idx0] = input->VirtualBoundaryP2X[idx0];
    dangerInput.VirtualBoundaryP2Y[idx0] = input->VirtualBoundaryP2Y[idx0];
    idx0 = idx0 + 1;
  }

  idx0 = 0;
  while (idx0 < DGR_PT_MAX) {
    dangerInput.BoundarySepPtX[idx0] = input->BoundarySepPtX[idx0];
    dangerInput.BoundarySepPtY[idx0] = input->BoundarySepPtY[idx0];
    dangerInput.StopperPtX[idx0] = input->StopperPtX[idx0];
    dangerInput.StopperPtY[idx0] = input->StopperPtY[idx0];
    idx0 = idx0 + 1;
  }

  geoVtDangerPt(&dangerInput, dangerOutput);
}

static void geoVtEntrPtDngPtTransformDanger(const GeoVtEntrPtDngPtInput_t *input,
                                            const GeoVtDangerPtOutput_t *dangerOutput,
                                            GeoVtEntrPtDngPtDangerRel_t *dangerRel) {
  Real cosT0;
  Real sinT0;
  Real dx0;
  Real dy0;

  cosT0 = cosf(input->TgtTheta);
  sinT0 = sinf(input->TgtTheta);
  dangerRel->RealX = 0.0f;
  dangerRel->RealY = 0.0f;
  dangerRel->VirtualX = 0.0f;
  dangerRel->VirtualY = 0.0f;

  if (dangerOutput->PtExReal > 0) {
    dx0 = dangerOutput->DangerPtRealX - input->TgtX;
    dy0 = dangerOutput->DangerPtRealY - input->TgtY;
    dangerRel->RealX = (cosT0 * dx0) + (sinT0 * dy0);
    dangerRel->RealY = (cosT0 * dy0) - (sinT0 * dx0);
  }

  if (dangerOutput->PtExVirtual > 0) {
    dx0 = dangerOutput->DangerPtVirtualX - input->TgtX;
    dy0 = dangerOutput->DangerPtVirtualY - input->TgtY;
    dangerRel->VirtualX = (cosT0 * dx0) + (sinT0 * dy0);
    dangerRel->VirtualY = (cosT0 * dy0) - (sinT0 * dx0);
  }
}

static Real geoVtEntrPtDngPtClearLeft(const GeoVtEntrPtDngPtInput_t *input,
                                      const GeoVtEntrPtDngPtGeometry_t *geom) {
  Real halfWidth0;
  Real openLimit0;
  Real enLlim0;
  Real rdSafe0;
  Real delta0;
  Real inside0;
  Real enLtmp0;
  Real candidate0;
  Real result0;
  Boolean condOpen0;
  Boolean condBlock0;
  Boolean condCannot0;

  halfWidth0 = GEO_VT_ENTR_VEH_WD * 0.5f;
  openLimit0 = halfWidth0 + 0.1f;
  condOpen0 = false;
  if (input->SurdType == GEO_VT_ENTR_SURD_NONE) {
    condOpen0 = true;
  } else {
    if (input->SurdType == GEO_VT_ENTR_SURD_RIGHT_REAR_ONLY) {
      if (geom->RightWidth > openLimit0) {
        condOpen0 = true;
      }
    }
  }

  if (condOpen0) {
    enLlim0 = GEO_VT_ENTR_LEN_SHORT - input->AgrOfst;
    result0 = apaMathMax(enLlim0, 0.3f);
  } else {
    enLlim0 = GEO_VT_ENTR_LEN_LONG - input->AgrOfst;
    enLlim0 = apaMathMax(enLlim0, 0.3f);
    condBlock0 = false;
    if (input->SurdType == GEO_VT_ENTR_SURD_RIGHT_REAR_ONLY) {
      if (geom->RightWidth < openLimit0) {
        condBlock0 = true;
      }
    } else {
      if (input->SurdType == GEO_VT_ENTR_SURD_BOTH) {
        if (geom->RightWidth < openLimit0) {
          condBlock0 = true;
        }
      }
    }
    condCannot0 = false;
    if (geom->LeftWidth < (halfWidth0 + GEO_VT_ENTR_LAT_OFST_REAL)) {
      condCannot0 = true;
    } else {
      if (condBlock0) {
        condCannot0 = true;
      }
    }
    if (condCannot0) {
      result0 = -1.0f;
    } else {
      rdSafe0 = input->Radius - halfWidth0 - GEO_VT_ENTR_LAT_OFST_REAL;
      delta0 = input->Radius - geom->LeftWidth;
      inside0 = (rdSafe0 * rdSafe0) - (delta0 * delta0);
      enLtmp0 = geoVtEntrPtDngPtSafeSqrt(inside0);
      candidate0 = geom->LeftFront - enLtmp0;
      result0 = apaMathMax(enLlim0, candidate0);
    }
  }

  return result0;
}

static Real geoVtEntrPtDngPtClearRight(const GeoVtEntrPtDngPtInput_t *input,
                                       const GeoVtEntrPtDngPtGeometry_t *geom) {
  Real halfWidth0;
  Real openLimit0;
  Real enLlim0;
  Real rdSafe0;
  Real delta0;
  Real inside0;
  Real enLtmp0;
  Real candidate0;
  Real result0;
  Boolean condOpen0;
  Boolean condBlock0;
  Boolean condCannot0;

  halfWidth0 = GEO_VT_ENTR_VEH_WD * 0.5f;
  openLimit0 = halfWidth0 + 0.1f;
  condOpen0 = false;
  if (input->SurdType == GEO_VT_ENTR_SURD_NONE) {
    condOpen0 = true;
  } else {
    if (input->SurdType == GEO_VT_ENTR_SURD_LEFT_FRONT_ONLY) {
      if (geom->LeftWidth > openLimit0) {
        condOpen0 = true;
      }
    }
  }

  if (condOpen0) {
    enLlim0 = GEO_VT_ENTR_LEN_SHORT - input->AgrOfst;
    result0 = apaMathMax(enLlim0, 0.3f);
  } else {
    enLlim0 = GEO_VT_ENTR_LEN_LONG - input->AgrOfst;
    enLlim0 = apaMathMax(enLlim0, 0.3f);
    condBlock0 = false;
    if (input->SurdType == GEO_VT_ENTR_SURD_LEFT_FRONT_ONLY) {
      if (geom->LeftWidth < openLimit0) {
        condBlock0 = true;
      }
    } else {
      if (input->SurdType == GEO_VT_ENTR_SURD_BOTH) {
        if (geom->LeftWidth < openLimit0) {
          condBlock0 = true;
        }
      }
    }
    condCannot0 = false;
    if (geom->RightWidth < (halfWidth0 + GEO_VT_ENTR_LAT_OFST_REAL)) {
      condCannot0 = true;
    } else {
      if (condBlock0) {
        condCannot0 = true;
      }
    }
    if (condCannot0) {
      result0 = -1.0f;
    } else {
      rdSafe0 = input->Radius - halfWidth0 - GEO_VT_ENTR_LAT_OFST_REAL;
      delta0 = input->Radius - geom->RightWidth;
      inside0 = (rdSafe0 * rdSafe0) - (delta0 * delta0);
      enLtmp0 = geoVtEntrPtDngPtSafeSqrt(inside0);
      candidate0 = geom->RightFront - enLtmp0;
      result0 = apaMathMax(enLlim0, candidate0);
    }
  }

  return result0;
}

static Real geoVtEntrPtDngPtDangerLeftLimit(const GeoVtEntrPtDngPtInput_t *input,
                                            const GeoVtEntrPtDngPtGeometry_t *geom,
                                            const GeoVtEntrPtDngPtDangerRel_t *dangerRel) {
  Real kLimFac0;
  Real depthLimit0;
  Real dangerLimit0;
  Real minLimit0;
  Real rawLimit0;
  Real absLat0;
  Real result0;

  if (input->SurdType == GEO_VT_ENTR_SURD_BOTH) {
    kLimFac0 = geoVtEntrPtDngPtLimitFactor(dangerRel->RealY);
    depthLimit0 = apaMathMin(geom->SltDepth, 3.3f * kLimFac0);
    depthLimit0 = depthLimit0 - input->AgrOfst;
    dangerLimit0 = dangerRel->RealX + 0.2f;
    minLimit0 = apaMathMin(dangerLimit0, depthLimit0);
    result0 = apaMathMax(minLimit0, 0.3f);
  } else {
    if (input->SurdType == GEO_VT_ENTR_SURD_LEFT_FRONT_ONLY) {
      kLimFac0 = geoVtEntrPtDngPtLimitFactor(dangerRel->RealY);
      rawLimit0 = (GEO_VT_ENTR_LEN_LONG * kLimFac0) - input->AgrOfst;
      result0 = apaMathMax(rawLimit0, 0.3f);
    } else {
      absLat0 = fabsf(dangerRel->RealY);
      if (absLat0 > GEO_VT_ENTR_LAT_DANGER_THD) {
        rawLimit0 = GEO_VT_ENTR_LEN_SHORT - input->AgrOfst;
      } else {
        if (dangerRel->RealX < 1.0f) {
          rawLimit0 = GEO_VT_ENTR_LEN_SHORT - input->AgrOfst;
        } else {
          rawLimit0 = GEO_VT_ENTR_LEN_MID - input->AgrOfst;
        }
      }
      result0 = apaMathMax(rawLimit0, 0.3f);
    }
  }

  return result0;
}

static Real geoVtEntrPtDngPtDangerRightLimit(const GeoVtEntrPtDngPtInput_t *input,
                                             const GeoVtEntrPtDngPtGeometry_t *geom,
                                             const GeoVtEntrPtDngPtDangerRel_t *dangerRel) {
  Real kLimFac0;
  Real depthLimit0;
  Real dangerLimit0;
  Real minLimit0;
  Real rawLimit0;
  Real absLat0;
  Real result0;

  if (input->SurdType == GEO_VT_ENTR_SURD_BOTH) {
    kLimFac0 = geoVtEntrPtDngPtLimitFactor(dangerRel->RealY);
    depthLimit0 = apaMathMin(geom->SltDepth, 3.3f * kLimFac0);
    depthLimit0 = depthLimit0 - input->AgrOfst;
    dangerLimit0 = dangerRel->RealX + 0.2f;
    minLimit0 = apaMathMin(dangerLimit0, depthLimit0);
    result0 = apaMathMax(minLimit0, 0.3f);
  } else {
    if (input->SurdType == GEO_VT_ENTR_SURD_RIGHT_REAR_ONLY) {
      kLimFac0 = geoVtEntrPtDngPtLimitFactor(dangerRel->RealY);
      rawLimit0 = (GEO_VT_ENTR_LEN_LONG * kLimFac0) - input->AgrOfst;
      result0 = apaMathMax(rawLimit0, 0.0f);
    } else {
      absLat0 = fabsf(dangerRel->RealY);
      if (absLat0 > GEO_VT_ENTR_LAT_DANGER_THD) {
        rawLimit0 = GEO_VT_ENTR_LEN_SHORT - input->AgrOfst;
      } else {
        if (dangerRel->RealX < 1.0f) {
          rawLimit0 = GEO_VT_ENTR_LEN_SHORT - input->AgrOfst;
        } else {
          rawLimit0 = GEO_VT_ENTR_LEN_MID - input->AgrOfst;
        }
      }
      result0 = apaMathMax(rawLimit0, 0.3f);
    }
  }

  return result0;
}

static Real geoVtEntrPtDngPtPlanAroundLat(
    Real radius, Real lateral, Real front, Real enLlim, Real latOfst, Boolean useAbsLat) {
  Real halfWidth0;
  Real lat0;
  Real checkLat0;
  Real rdSafe0;
  Real delta0;
  Real inside0;
  Real enLtmp0;
  Real candidate0;
  Real result0;

  halfWidth0 = GEO_VT_ENTR_VEH_WD * 0.5f;
  if (useAbsLat) {
    lat0 = fabsf(lateral);
    checkLat0 = lat0;
  } else {
    lat0 = lateral;
    checkLat0 = lateral;
  }
  if (checkLat0 < (halfWidth0 + latOfst)) {
    result0 = apaMathMax(front, enLlim);
  } else {
    rdSafe0 = radius - halfWidth0 - latOfst;
    delta0 = radius - lat0;
    inside0 = (rdSafe0 * rdSafe0) - (delta0 * delta0);
    enLtmp0 = geoVtEntrPtDngPtSafeSqrt(inside0);
    candidate0 = front - enLtmp0;
    result0 = apaMathMax(enLlim, candidate0);
  }

  return result0;
}

static Real geoVtEntrPtDngPtDangerLeft(const GeoVtEntrPtDngPtInput_t *input,
                                       const GeoVtEntrPtDngPtGeometry_t *geom,
                                       const GeoVtEntrPtDngPtDangerRel_t *dangerRel,
                                       Integer ptEx) {
  Real halfWidth0;
  Real enLlim0;
  Real result0;
  Boolean condRightBlocked0;
  Boolean condFar0;

  halfWidth0 = GEO_VT_ENTR_VEH_WD * 0.5f;
  enLlim0 = geoVtEntrPtDngPtDangerLeftLimit(input, geom, dangerRel);
  condRightBlocked0 = false;
  if (input->SurdType == GEO_VT_ENTR_SURD_RIGHT_REAR_ONLY) {
    if (geom->RightWidth < (halfWidth0 + 0.1f)) {
      condRightBlocked0 = true;
    }
  } else {
    if (input->SurdType == GEO_VT_ENTR_SURD_BOTH) {
      if (geom->RightWidth < (halfWidth0 + 0.1f)) {
        condRightBlocked0 = true;
      }
    }
  }

  if (condRightBlocked0) {
    result0 = -1.0f;
  } else {
    condFar0 = false;
    if (dangerRel->RealY > input->Radius) {
      condFar0 = true;
    } else {
      if (dangerRel->RealX < GEO_VT_ENTR_LEN_SHORT) {
        condFar0 = true;
      }
    }

    if (condFar0) {
      result0 = enLlim0;
    } else {
      if (ptEx == 2) {
        result0 = geoVtEntrPtDngPtPlanAroundLat(input->Radius,
                                                dangerRel->VirtualY,
                                                dangerRel->VirtualX,
                                                enLlim0,
                                                GEO_VT_ENTR_LAT_OFST_VIRTUAL,
                                                true);
      } else {
        result0 = geoVtEntrPtDngPtPlanAroundLat(input->Radius,
                                                dangerRel->RealY,
                                                dangerRel->RealX,
                                                enLlim0,
                                                GEO_VT_ENTR_LAT_OFST_REAL,
                                                false);
      }
    }
  }

  return result0;
}

static Real geoVtEntrPtDngPtDangerRight(const GeoVtEntrPtDngPtInput_t *input,
                                        const GeoVtEntrPtDngPtGeometry_t *geom,
                                        const GeoVtEntrPtDngPtDangerRel_t *dangerRel,
                                        Integer ptEx) {
  Real halfWidth0;
  Real enLlim0;
  Real absRealLat0;
  Real result0;
  Boolean condLeftBlocked0;
  Boolean condFar0;

  halfWidth0 = GEO_VT_ENTR_VEH_WD * 0.5f;
  enLlim0 = geoVtEntrPtDngPtDangerRightLimit(input, geom, dangerRel);
  condLeftBlocked0 = false;
  if (input->SurdType == GEO_VT_ENTR_SURD_LEFT_FRONT_ONLY) {
    if (geom->LeftWidth < (halfWidth0 + 0.1f)) {
      condLeftBlocked0 = true;
    }
  } else {
    if (input->SurdType == GEO_VT_ENTR_SURD_BOTH) {
      if (geom->LeftWidth < (halfWidth0 + 0.1f)) {
        condLeftBlocked0 = true;
      }
    }
  }

  if (condLeftBlocked0) {
    result0 = -1.0f;
  } else {
    absRealLat0 = fabsf(dangerRel->RealY);
    condFar0 = false;
    if (absRealLat0 > input->Radius) {
      condFar0 = true;
    } else {
      if (dangerRel->RealX < GEO_VT_ENTR_LEN_SHORT) {
        condFar0 = true;
      }
    }

    if (condFar0) {
      result0 = enLlim0;
    } else {
      if (ptEx == 2) {
        result0 = geoVtEntrPtDngPtPlanAroundLat(input->Radius,
                                                dangerRel->VirtualY,
                                                dangerRel->VirtualX,
                                                enLlim0,
                                                GEO_VT_ENTR_LAT_OFST_VIRTUAL,
                                                true);
      } else {
        result0 = geoVtEntrPtDngPtPlanAroundLat(input->Radius,
                                                dangerRel->RealY,
                                                dangerRel->RealX,
                                                enLlim0,
                                                GEO_VT_ENTR_LAT_OFST_REAL,
                                                true);
      }
    }
  }

  return result0;
}

static Real geoVtEntrPtDngPtCalcPlanLen(const GeoVtEntrPtDngPtInput_t *input,
                                        const GeoVtEntrPtDngPtGeometry_t *geom,
                                        const GeoVtEntrPtDngPtDangerRel_t *dangerRel,
                                        Integer ptEx) {
  Real planLen0;

  if (ptEx == 0) {
    if (input->PrkType == GEO_VT_ENTR_PRK_LEFT) {
      planLen0 = geoVtEntrPtDngPtClearLeft(input, geom);
    } else {
      if (input->PrkType == GEO_VT_ENTR_PRK_RIGHT) {
        planLen0 = geoVtEntrPtDngPtClearRight(input, geom);
      } else {
        planLen0 = -1.0f;
      }
    }
  } else {
    if (input->PrkType == GEO_VT_ENTR_PRK_LEFT) {
      planLen0 = geoVtEntrPtDngPtDangerLeft(input, geom, dangerRel, ptEx);
    } else {
      if (input->PrkType == GEO_VT_ENTR_PRK_RIGHT) {
        planLen0 = geoVtEntrPtDngPtDangerRight(input, geom, dangerRel, ptEx);
      } else {
        planLen0 = -1.0f;
      }
    }
  }

  return planLen0;
}

/**
 * @brief 计算VT入口边中点、危险点及危险约束下的规划入口点
 * @en_name geoVtEntrPtDngPt
 * @cn_name VT入口点危险点检测
 * @type module
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TgtX 目标X
 * @param[IN] Real TgtY 目标Y
 * @param[IN] Real TgtTheta 目标航向
 * @param[IN] Real Radius 规划转弯半径
 * @param[IN] Integer PrkType 泊车方向1左2右
 * @param[IN] Integer SurdType 周边类型0双侧受限1仅右后受限2仅左前受限3无障碍
 * @param[IN] Real AgrOfst 可用空间修正量
 * @param[IN] Integer BoundaryLineNum 真实边界线数量
 * @param[IN] Real[64] BoundaryP1X 真实边界点1X
 * @param[IN] Real[64] BoundaryP1Y 真实边界点1Y
 * @param[IN] Real[64] BoundaryP2X 真实边界点2X
 * @param[IN] Real[64] BoundaryP2Y 真实边界点2Y
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线数量
 * @param[IN] Real[64] VirtualBoundaryP1X 虚拟边界点1X
 * @param[IN] Real[64] VirtualBoundaryP1Y 虚拟边界点1Y
 * @param[IN] Real[64] VirtualBoundaryP2X 虚拟边界点2X
 * @param[IN] Real[64] VirtualBoundaryP2Y 虚拟边界点2Y
 * @param[IN] Integer BoundarySepPtNum 真实离散点数量
 * @param[IN] Real[128] BoundarySepPtX 真实离散点X
 * @param[IN] Real[128] BoundarySepPtY 真实离散点Y
 * @param[IN] Integer StopperPtNum 挡停点数量
 * @param[IN] Real[128] StopperPtX 挡停点X
 * @param[IN] Real[128] StopperPtY 挡停点Y
 * @param[OUT] Real EntrX 入口点X
 * @param[OUT] Real EntrY 入口点Y
 * @param[OUT] Real EntrPlanX 规划入口点X
 * @param[OUT] Real EntrPlanY 规划入口点Y
 * @param[OUT] Integer DgrPtFlg 危险点标志0无1真实2虚拟
 * @param[OUT] Real DgrPtX 危险点X
 * @param[OUT] Real DgrPtY 危险点Y
 * @param[OUT] Real DgrPtRX 危险点目标坐标系X
 * @param[OUT] Real DgrPtRY 危险点目标坐标系Y
 * @param[OUT] Integer CheckCount 有效检查次数
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite geo_vt_entrPt_dngPt.m
 * @version 1.1
 * @date 2026-03-04
 * @author aide
 */
void geoVtEntrPtDngPt(const GeoVtEntrPtDngPtInput_t *input, GeoVtEntrPtDngPtOutput_t *output) {
  GeoVtEntrPtDngPtGeometry_t geom;
  GeoVtDangerPtOutput_t dangerOutput;
  GeoVtEntrPtDngPtDangerRel_t dangerRel;
  Real planLen0;
  Real cosT0;
  Real sinT0;
  Real midX0;
  Real midY0;
  Real dgrPtX0;
  Real dgrPtY0;
  Real dgrPtRX0;
  Real dgrPtRY0;

  geom = (GeoVtEntrPtDngPtGeometry_t){0};
  dangerOutput = (GeoVtDangerPtOutput_t){0};
  dangerRel = (GeoVtEntrPtDngPtDangerRel_t){0};

  geoVtEntrPtDngPtCalcGeometry(input, &geom);
  geoVtEntrPtDngPtCallDanger(input, &geom, &dangerOutput);
  geoVtEntrPtDngPtTransformDanger(input, &dangerOutput, &dangerRel);
  planLen0 = geoVtEntrPtDngPtCalcPlanLen(input, &geom, &dangerRel, dangerOutput.PtEx);

  cosT0 = cosf(input->TgtTheta);
  sinT0 = sinf(input->TgtTheta);
  if (planLen0 >= 0.0f) {
    output->EntrPlanX = input->TgtX + (planLen0 * cosT0);
    output->EntrPlanY = input->TgtY + (planLen0 * sinT0);
  } else {
    output->EntrPlanX = geom.CrossX;
    output->EntrPlanY = geom.CrossY;
  }

  midX0 = (geom.EntrP1X + geom.EntrP2X) * 0.5f;
  midY0 = (geom.EntrP1Y + geom.EntrP2Y) * 0.5f;
  if (dangerOutput.PtExReal > 0) {
    dgrPtX0 = dangerOutput.DangerPtRealX;
    dgrPtY0 = dangerOutput.DangerPtRealY;
    dgrPtRX0 = dangerRel.RealX;
    dgrPtRY0 = dangerRel.RealY;
  } else {
    dgrPtX0 = 0.0f;
    dgrPtY0 = 0.0f;
    dgrPtRX0 = 0.0f;
    dgrPtRY0 = 0.0f;
  }
  output->EntrX = midX0;
  output->EntrY = midY0;
  output->DgrPtFlg = dangerOutput.PtEx;
  output->DgrPtX = dgrPtX0;
  output->DgrPtY = dgrPtY0;
  output->DgrPtRX = dgrPtRX0;
  output->DgrPtRY = dgrPtRY0;
  output->CheckCount = dangerOutput.CheckCount;
}

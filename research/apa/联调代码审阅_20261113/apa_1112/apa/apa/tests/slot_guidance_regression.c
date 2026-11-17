#include "apaPlanningSlotGuidance.h"
#include <assert.h>
#include <string.h>

static VehicleCollisionCheckInput_t slot_input(Real rear_y, Real yaw) {
  VehicleCollisionCheckInput_t input;
  const Real points[8] = {-2, 0, 2, 0, 2, 8, -2, 8};
  memset(&input, 0, sizeof(input));
  memcpy(input.SlotPts, points, sizeof(points));
  input.SlotPtsValid = true;
  input.VehLF = 3.6f;
  input.VehLB = 0.8f;
  input.VehW = 1.8f;
  input.CheckType = VCC_CHECK_TYPE_PLANNING;
  input.SlotClass = VCC_SLOT_CLASS_VERTICAL;
  input.VehPosY = rear_y;
  input.VehPosTheta = yaw;
  input.VirtualBoundaryLineNum = 1;
  return input;
}

static void entry_reference(void) {
  const Real half_pi = 1.57079632679f;
  VehicleCollisionCheckInput_t front = slot_input(-0.9f, half_pi);
  VehicleCollisionCheckInput_t rear = slot_input(1.9f, -half_pi);
  assert(apaPlanningSlotBodyCenterIsInside(&front));
  assert(apaPlanningSlotBodyCenterIsInside(&rear));
  front.VehPosY = -1.9f;
  rear.VehPosY = 0.9f;
  assert(!apaPlanningSlotBodyCenterIsInside(&front));
  assert(!apaPlanningSlotBodyCenterIsInside(&rear));
}

static void side_clipping(void) {
  for (int reversed = 0; reversed < 2; ++reversed) {
    VehicleCollisionCheckInput_t input = slot_input(2, 0), soft = input;
    input.VirtualBoundaryLineX1[0] = input.VirtualBoundaryLineX2[0] = 2.005f;
    input.VirtualBoundaryLineY1[0] = reversed ? 10 : -1;
    input.VirtualBoundaryLineY2[0] = reversed ? -1 : 10;
    apaPlanningSlotSoftVirtualLines(&input, &soft);
    assert(soft.VirtualBoundaryLineNum == 1);
    assert(fabs(soft.VirtualBoundaryLineX1[0] - 2.005) < 1.e-6);
    assert(fabs(soft.VirtualBoundaryLineX2[0] - 2.005) < 1.e-6);
    assert(fabs(soft.VirtualBoundaryLineY1[0]) < 1.e-6);
    assert(fabs(soft.VirtualBoundaryLineY2[0] - 8) < 1.e-6);
  }
}

static void degenerate_projections(void) {
  const Real lines[][4] = {{1.995f, 2, 2.005f, 2}, {2, 2, 2, 2},
                           {1.995f, 2, 2.005f, 2.0002f}, {NAN, 2, 2, 3}};
  for (unsigned int i = 0; i < sizeof(lines) / sizeof(lines[0]); ++i) {
    VehicleCollisionCheckInput_t input = slot_input(2, 0), soft = input;
    input.VirtualBoundaryLineX1[0] = lines[i][0];
    input.VirtualBoundaryLineY1[0] = lines[i][1];
    input.VirtualBoundaryLineX2[0] = lines[i][2];
    input.VirtualBoundaryLineY2[0] = lines[i][3];
    apaPlanningSlotSoftVirtualLines(&input, &soft);
    for (Integer j = 0; j < soft.VirtualBoundaryLineNum; ++j) {
      assert(isfinite(soft.VirtualBoundaryLineX1[j]));
      assert(isfinite(soft.VirtualBoundaryLineY1[j]));
      assert(isfinite(soft.VirtualBoundaryLineX2[j]));
      assert(isfinite(soft.VirtualBoundaryLineY2[j]));
    }
  }
}

static void straight_exit_geometry(void) {
  const Real half_pi = 1.57079632679f;
  for (int head_in = 0; head_in < 2; ++head_in) {
    VehicleCollisionCheckInput_t input = slot_input(3, head_in ? half_pi : -half_pi);
    const double distance = apaPlanningSlotStraightExitDistance(&input);
    assert(head_in ? distance < 0 : distance > 0);
    /* No target is needed, and park-out must retain the same seed geometry. */
    input.ParkingOutFlag = true;
    assert(apaPlanningSlotDepthWeight(&input) == 0);
    assert(fabs(apaPlanningSlotStraightExitDistance(&input) - distance) < 1.e-9);
    const double end_x = input.VehPosX + distance * cos(input.VehPosTheta);
    const double end_y = input.VehPosY + distance * sin(input.VehPosTheta);
    double furthest = -INFINITY;
    for (int front = 0; front < 2; ++front)
      for (int left = 0; left < 2; ++left) {
        const double longitudinal = front ? input.VehLF : -input.VehLB;
        const double lateral = (left ? 0.5 : -0.5) * input.VehW;
        const double y = end_y + longitudinal * sin(input.VehPosTheta) +
                         lateral * cos(input.VehPosTheta);
        furthest = fmax(furthest, y);
      }
    assert(fabs(furthest) < 1.e-6);
    /* Rigid transforms and entrance winding must preserve gear and distance. */
    const double angle = 0.7, c = cos(angle), s = sin(angle);
    for (int i = 0; i < 4; ++i) {
      const double x = input.SlotPts[2 * i], y = input.SlotPts[2 * i + 1];
      input.SlotPts[2 * i] = 17 + c * x - s * y;
      input.SlotPts[2 * i + 1] = -23 + s * x + c * y;
    }
    input.VehPosX = 17 - 3 * s;
    input.VehPosY = -23 + 3 * c;
    input.VehPosTheta += angle;
    assert(fabs(apaPlanningSlotStraightExitDistance(&input) - distance) < 1.e-5);
    const Real reversed[8] = {input.SlotPts[2], input.SlotPts[3],
                              input.SlotPts[0], input.SlotPts[1],
                              input.SlotPts[6], input.SlotPts[7],
                              input.SlotPts[4], input.SlotPts[5]};
    memcpy(input.SlotPts, reversed, sizeof(reversed));
    assert(fabs(apaPlanningSlotStraightExitDistance(&input) - distance) < 1.e-5);
    assert(isfinite(end_x));
  }
  VehicleCollisionCheckInput_t input = slot_input(3, half_pi);
  input.SlotClass = VCC_SLOT_CLASS_PARALLEL;
  assert(apaPlanningSlotStraightExitDistance(&input) == 0);
  input.SlotClass = VCC_SLOT_CLASS_VERTICAL;
  input.VehPosTheta = 0;
  assert(apaPlanningSlotStraightExitDistance(&input) == 0);
  input.VehPosTheta = half_pi;
  input.VehPosY = -5;
  assert(apaPlanningSlotStraightExitDistance(&input) == 0);
  input.VehPosY = 3;
  input.SlotPts[2] = input.SlotPts[0];
  input.SlotPts[3] = input.SlotPts[1];
  assert(apaPlanningSlotStraightExitDistance(&input) == 0);
  input.SlotPts[0] = NAN;
  assert(apaPlanningSlotStraightExitDistance(&input) == 0);
}

int main(void) {
  entry_reference();
  side_clipping();
  degenerate_projections();
  straight_exit_geometry();
  return 0;
}

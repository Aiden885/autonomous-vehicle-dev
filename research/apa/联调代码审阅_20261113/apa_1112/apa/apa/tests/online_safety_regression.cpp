#include "apa_online_safety.h"
#include "apaMocoExecution.h"
#include "apaVehicleDefaults.h"
#include <cassert>

using namespace pangu::modules;

namespace {
ApaInput Scene(double x, double y, double vy = 0.0) {
  ApaInput input;
  input.mutable_scene()->set_version(10);
  auto* obstacle = input.mutable_scene()->add_obstacles();
  obstacle->set_id("actor:12");
  obstacle->set_velocity_y(vy);
  for (const auto& xy : {std::pair{x-0.1, y-0.1}, {x+0.1, y-0.1}, {x+0.1, y+0.1}, {x-0.1, y+0.1}}) {
    auto* p = obstacle->add_footprint(); p->set_x(xy.first); p->set_y(xy.second);
  }
  return input;
}
}

void CheckOnlineSafetyCases() {
  ApaMocoCtrlChartV2State_t chart{};
  assert(CheckApaOnlineSafety(Scene(2, 0), nullptr, chart, 0, 0).reason() == "body_blocked");
  assert(!CheckApaOnlineSafety(Scene(2, 4), nullptr, chart, 0, 0).blocked());
  const auto predicted = CheckApaOnlineSafety(Scene(2, 4, -3), nullptr, chart, 0, 0);
  assert(predicted.blocked() && predicted.reason() == "body_predicted");
  chart.Phase = APA_MOCO_DONE;
  assert(!CheckApaOnlineSafety(Scene(2, 4, -3), nullptr, chart, 0, 0).blocked());
  assert(CheckApaOnlineSafety(Scene(2, 0), nullptr, chart, 0, 0).reason() == "body_blocked");
  chart.Phase = APA_MOCO_TRACK;
  auto enclosing = Scene(0, 0);
  auto* box = enclosing.mutable_scene()->mutable_obstacles(0);
  for (auto& p : *box->mutable_footprint()) { p.set_x(p.x()*100); p.set_y(p.y()*100); }
  assert(CheckApaOnlineSafety(enclosing, nullptr, chart, 0, 0).blocked());

  ::apa::ApaParkingPathPlanningOutputData path{};
  path.pathPointCount = 2;
  path.pathPointX[1] = 10.0f;
  chart.StpNum = 1; chart.StpNow = 0; chart.SegEndIdx[0] = 1;
  const auto blocked = CheckApaOnlineSafety(Scene(5, 0), &path, chart, 0.5, 1.0);
  assert(blocked.blocked() && blocked.reason() == "path_blocked" && blocked.scene_version() == 10);
  assert(!CheckApaOnlineSafety(Scene(5, 3), &path, chart, 0.5, 1.0).blocked());
  assert(!CheckApaOnlineSafety(Scene(8, 0), &path, chart, 0.5, 1.0).blocked());
  assert(CheckApaCandidatePath(Scene(8, 0), path).blocked());
  assert(CheckApaOnlineSafety(Scene(8, 0), &path, chart, 5.0, 1.0).blocked());

  path.pathPointX[1] = -10.0f;
  assert(CheckApaOnlineSafety(Scene(-2, 0), &path, chart, 0.5, 1.0).blocked());
  path.pathPointX[1] = 0.5f; path.pathPointX[2] = -3.0f; path.pathPointCount = 3;
  chart.StpNum = 2; chart.SegStartIdx[1] = 1; chart.SegEndIdx[1] = 2;
  assert(!CheckApaOnlineSafety(Scene(-2, 0), &path, chart, 0.5, 1.0).blocked());
  assert(CheckApaCandidatePath(Scene(-2, 0), path).blocked());
  chart.StpNow = 1;
  assert(CheckApaOnlineSafety(Scene(-2, 0), &path, chart, 0.5, 1.0).blocked());

  auto line = Scene(20, 20);
  line.set_boundary_line_num(1);
  line.add_boundary_line_x1(2.0); line.add_boundary_line_y1(-2.0);
  line.add_boundary_line_x2(2.0); line.add_boundary_line_y2(2.0);
  assert(CheckApaOnlineSafety(line, nullptr, chart, 0, 0).obstacle_id() == "json:boundary:0");

  auto marking = Scene(20, 20);
  const double side = APA_VEHICLE_WIDTH_M/2.0;
  marking.set_virtual_boundary_line_num(1);
  marking.add_virtual_boundary_p1_x(0.0); marking.add_virtual_boundary_p1_y(side+0.02);
  marking.add_virtual_boundary_p2_x(2.0); marking.add_virtual_boundary_p2_y(side+0.02);
  assert(!CheckApaOnlineSafety(marking, nullptr, chart, 0, 0).blocked());
  marking.set_boundary_line_num(1);
  marking.add_boundary_line_x1(0.0); marking.add_boundary_line_y1(side+0.02);
  marking.add_boundary_line_x2(2.0); marking.add_boundary_line_y2(side+0.02);
  const auto clearance = CheckApaOnlineSafety(marking, nullptr, chart, 0, 0);
  assert(clearance.reason() == "planning_clearance" && clearance.obstacle_id() == "json:boundary:0");
  assert(IsApaRecoverableObstacle(clearance.reason()));
  assert(!IsApaRecoverableObstacle("body_blocked"));
  assert(CheckApaOnlineSafety(Scene(2, 1.20), nullptr, chart, 0, 0).reason() == "planning_clearance");

  auto stopper = Scene(20, 20);
  stopper.set_slot_track_stopper_num(1);
  stopper.add_slot_track_stopper_x1(-0.8); stopper.add_slot_track_stopper_y1(-0.5);
  stopper.add_slot_track_stopper_x2(-0.8); stopper.add_slot_track_stopper_y2(0.5);
  assert(!CheckApaOnlineSafety(stopper, nullptr, chart, 0, 0).blocked());
  stopper.set_slot_track_stopper_x1(0, 1.0); stopper.set_slot_track_stopper_x2(0, 1.0);
  const auto wheel_hit = CheckApaOnlineSafety(stopper, nullptr, chart, 0, 0);
  assert(wheel_hit.reason() == "body_blocked" && wheel_hit.obstacle_id() == "json:slot_stopper:0");

  auto numbered = Scene(20, 20);
  numbered.set_virtual_boundary_line_num(2);
  for (double y : {4.0, 0.5}) {
    numbered.add_virtual_boundary_p1_x(0.0); numbered.add_virtual_boundary_p1_y(y);
    numbered.add_virtual_boundary_p2_x(2.0); numbered.add_virtual_boundary_p2_y(y);
  }
  assert(CheckApaOnlineSafety(numbered, nullptr, chart, 0, 0).obstacle_id() == "json:virtual:1");
}

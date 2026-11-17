extern "C" {
#include "apaHybridAStar.h"
#include "apaParkingSearch.h"
}
#include "apa_parking_motion.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
using apa::parking::Pose;
using apa::parking::Control;
using apa::parking::RsCollection;
using apa::parking::advance;
using apa::parking::wrap;
using apa::parking::append;

constexpr double kXy = .04, kYaw = kPi / 180.0;

constexpr int kExpandedLimit = 40000, kLabelLimit = 250000;
constexpr int kQualityExpansionLimit = 600;

constexpr int kSearchControlLimit = 30;
constexpr int kNormalizationTrialLimit = 2048;
constexpr double kHeuristicWeight = 1.5;
constexpr double kFieldStep = .20, kFieldRange = 24.0;
enum class Resolution { Nominal, Detailed, Region, Tails, Fine };
constexpr std::array<double, 5> kLengths{.08, .16, .20, .32, .64};
constexpr int kNominalLengthIndex = 3;

constexpr int kNominalExpansionLimit = kExpandedLimit / 4;
constexpr int kNominalLabelLimit = kLabelLimit / 4;
constexpr int kDetailedExpansionLimit = 3000;
double pointSegment(double x, double y, double ax, double ay, double bx,
                    double by) {
  const double dx = bx - ax, dy = by - ay, n = dx * dx + dy * dy;
  const double t =
      n > 0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / n, 0.0, 1.0) : 0;
  return std::hypot(x - ax - t * dx, y - ay - t * dy);
}
struct Key {
  int x, y, yaw, gear, steer, root;
  bool operator==(const Key &o) const {
    return x == o.x && y == o.y && yaw == o.yaw && gear == o.gear &&
           steer == o.steer && root == o.root;
  }
};
struct Hash {
  size_t operator()(const Key &k) const {
    uint64_t h = 1469598103934665603ULL;
    for (int v : {k.x, k.y, k.yaw, k.gear, k.steer, k.root})
      h = (h ^ uint32_t(v)) * 1099511628211ULL;
    return h;
  }
};
struct Node {
  Pose pose;
  double g, h, distance;
  int parent, gear, steer, controls;
  bool informed = false;
  ApaParkingRunQuality_t quality{};
  int root = 0;
  double previousK = 0;
  double guideG = 0;
};
struct Label {
  int node;
  bool closed;
  bool guide, quality;
};
struct Entry {
  double f;
  int index;
  bool operator<(const Entry &o) const {
    return f == o.f ? index > o.index : f > o.f;
  }
};
class Search {
  ApaHybridAStar_t &planner;
  VehicleCollisionCheckInput_t environment;
  ApaPlanningSceneQuery_t scene;
  const double radius, curvature;
  std::array<Real, 3> margins{}, minimumMargins{};
  double corner = 0;
  Pose worldOrigin;
  double frameCos, frameSin;
  Pose start, goal;
  std::vector<Pose> roots;
  std::vector<std::vector<Control>> rootPrefixes;
  size_t anchorCount = 0;
  std::vector<Node> nodes;
  std::unordered_map<Key, std::vector<Label>, Hash> labels;
  std::priority_queue<Entry> open;
  std::priority_queue<Entry> qualityOpen;
  std::vector<double> field;
  std::vector<unsigned char> occupied;
  int fieldWidth = 0;
  double fieldX = 0, fieldY = 0;
  double bestCost = std::numeric_limits<double>::infinity();
  ApaParkingQuality_t bestQuality{INT_MAX,INT_MAX,std::numeric_limits<double>::infinity()};
  int firstSolution = -1, expanded = 0, reopened = 0, generated = 0,
      dominated = 0;
  size_t checks = 0, connections = 0;
  size_t connectionBlocked=0, connectionLengthRejected=0, capacityRejected=0,
      normalizationTrials=0, normalizedCandidates=0, controlPruned=0;
  struct CompletedRoute {
    std::vector<Control> controls;
    double length;
    int root;
  };
  std::vector<CompletedRoute> overflowCandidates;
  size_t oversizedConnections=0, capacityPoolDropped=0;
  bool memoryFull = false;
  bool executionBudgetFull = false;
  int executionChecks = 0;
  int qualityOnlyLabels = 0, qualityCapacityRejected = 0, qualityExpansions = 0;
  Resolution resolution = Resolution::Nominal;
  size_t retiredLabels = 0;
  int phaseFirstSolution = -1;
  int improvementExpansions = 0;
  ApaParkingQuality_t phaseIncumbent{};
  double executionSeconds = 0;
  double firstSolutionSeconds=0, firstExecutionSeconds=0;
  std::chrono::steady_clock::time_point searchBegan;
  const char *reason = "open_empty";

  void markFirstExecutable() {
    if (firstSolution>=0) return;
    firstSolution=expanded;
    firstSolutionSeconds=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-searchBegan).count();
    firstExecutionSeconds=executionSeconds;
  }

  Pose toWorld(Pose p) const {
    return {worldOrigin.x + frameCos * p.x - frameSin * p.y,
            worldOrigin.y + frameSin * p.x + frameCos * p.y,
            wrap(worldOrigin.yaw + p.yaw)};
  }

  Key key(const Node &n) const {
    int a = int(std::llround(wrap(n.pose.yaw) / kYaw));
    if (a < 0)
      a += 360;
    return {int(std::llround((n.pose.x - start.x) / kXy)),
            int(std::llround((n.pose.y - start.y) / kXy)), a % 360, n.gear,
            n.steer,n.root};
  }
  bool clear(Pose p, double sweepBound = 0) {
    ++checks;
    p = toWorld(p);
    environment.VehPosX = p.x;
    environment.VehPosY = p.y;
    environment.VehPosTheta = p.yaw;
    environment.UseAutoMargin = false;
    environment.FrontSafeMargin = margins[0] + sweepBound;
    environment.RearSafeMargin = margins[1] + sweepBound;
    environment.SideSafeMargin = margins[2] + sweepBound;
    environment.WhlBseLongMargin = APA_PARKIN_PHYSICAL_MARGIN_M + sweepBound;
    environment.WhlBseLatMargin = APA_PARKIN_PHYSICAL_MARGIN_M + sweepBound;
    VehicleCollisionCheckOutput_t result{};
    apaPlanningSceneCollisionCheck(&environment, &scene, &result);
    return !result.IsCollision;
  }
  bool arc(Pose p, double distance, double k, int depth = 0) {

    const Pose mid = advance(p, distance / 2, k);
    const double bound = std::abs(distance) / 2 * (1 + corner * std::abs(k));
    const Pose world = toWorld(mid);
    const Real x = world.x, y = world.y;
    const double rounding =
        4 * std::max(double(std::nextafter(x, INFINITY) - x),
                     double(std::nextafter(y, INFINITY) - y)) +
        1.e-6;
    if (clear(mid, bound + rounding))
      return true;
    if (!clear(mid) || bound <= rounding || depth >= 20)
      return false;
    return arc(p, distance / 2, k, depth + 1) &&
           arc(mid, distance / 2, k, depth + 1);
  }
  double clearanceCost(Pose p, double distance, double k, double* guidance=nullptr) {
    const int count = std::max(1, int(std::ceil(std::abs(distance) / .16)));
    double cost = 0;
    double guide = 0;
    for (int i = 0; i < count; ++i) {
      const Pose world = toWorld(advance(p, distance * (i + .5) / count, k));
      auto envelope = environment;
      envelope.VehPosX = world.x;
      envelope.VehPosY = world.y;
      envelope.VehPosTheta = world.yaw;
      envelope.UseAutoMargin = false;

      envelope.FrontSafeMargin = minimumMargins[0];
      envelope.RearSafeMargin = minimumMargins[1];
      envelope.SideSafeMargin = minimumMargins[2];
      envelope.WhlBseLongMargin = envelope.WhlBseLatMargin = APA_PARKIN_PHYSICAL_MARGIN_M;
      cost += apaPlanningSceneClearancePenalty(&envelope, &scene);
      if (guidance) guide+=apaPlanningSceneFeasibilityPenalty(&envelope,&scene);
    }
    if (guidance) *guidance=std::abs(distance)*guide/count;
    return std::abs(distance) * cost / count;
  }
  bool disk(double x, double y, double r) {
    const Pose world = toWorld({x, y, 0});
    x = world.x;
    y = world.y;
    if (scene.CheckDisk && scene.CheckDisk(x, y, r, scene.UserData))
      return true;
    for (int i = 0; i < environment.BoundaryLineNum; ++i)
      if (pointSegment(x, y, environment.BoundaryLineX1[i],
                       environment.BoundaryLineY1[i],
                       environment.BoundaryLineX2[i],
                       environment.BoundaryLineY2[i]) < r)
        return true;
    for (int i = 0; i < environment.VirtualBoundaryLineNum; ++i)
      if (pointSegment(x, y, environment.VirtualBoundaryLineX1[i],
                       environment.VirtualBoundaryLineY1[i],
                       environment.VirtualBoundaryLineX2[i],
                       environment.VirtualBoundaryLineY2[i]) < r)
        return true;
    return false;
  }
  int fieldIndex(Pose p) const {
    const int x = int(std::floor((p.x - fieldX) / kFieldStep));
    const int y = int(std::floor((p.y - fieldY) / kFieldStep));
    return x >= 0 && y >= 0 && x < fieldWidth && y < fieldWidth
               ? y * fieldWidth + x
               : -1;
  }
  void buildField() {
    fieldWidth = int(2 * kFieldRange / kFieldStep) + 1;
    fieldX = (start.x + goal.x) / 2 - kFieldRange;
    fieldY = (start.y + goal.y) / 2 - kFieldRange;
    field.assign(fieldWidth * fieldWidth,
                 std::numeric_limits<double>::infinity());
    occupied.assign(field.size(), 0);

    const double r = std::max(
        0.0, std::min(double(environment.VehLB), double(environment.VehW) / 2) +
                 std::min(APA_PARKIN_MARGIN_M, APA_PARKIN_BACK_MARGIN_M) - kFieldStep / std::sqrt(2.0));
    for (int y = 0; y < fieldWidth; ++y)
      for (int x = 0; x < fieldWidth; ++x)
        occupied[y * fieldWidth + x] = disk(fieldX + (x + .5) * kFieldStep,
                                            fieldY + (y + .5) * kFieldStep, r);
    const int target = fieldIndex(goal);
    if (target < 0)
      return;
    std::priority_queue<Entry> queue;
    field[target] = 0;
    occupied[target] = 0;
    queue.push({0, target});
    while (!queue.empty()) {
      const auto top = queue.top();
      queue.pop();
      if (top.f > field[top.index])
        continue;
      const int x = top.index % fieldWidth, y = top.index / fieldWidth;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          if ((!dx && !dy) || x + dx < 0 || y + dy < 0 ||
              x + dx >= fieldWidth || y + dy >= fieldWidth)
            continue;
          const int next = (y + dy) * fieldWidth + x + dx;
          if (occupied[next])
            continue;
          const double value =
              top.f + kFieldStep * (dx && dy ? std::sqrt(2.0) : 1.0);
          if (value < field[next]) {
            field[next] = value;
            queue.push({value, next});
          }
        }
    }
  }
  double heuristic(Pose p) const {
    double h = std::max(std::hypot(p.x - goal.x, p.y - goal.y),
                        radius * std::abs(wrap(p.yaw - goal.yaw)));
    const int i = fieldIndex(p);

    if (i >= 0 && std::isfinite(field[i]))
      h = std::max(h, field[i] - 2 * kFieldStep);
    return h;
  }
  RsCollection rs(Pose p) const {
    return apa::parking::connections(p,goal,radius);
  }
  double increment(double distance, double k, int gear,
                   double previousK) const {
    return apaParkingMotionCost(distance,k,previousK,gear,curvature);
  }
  double rsCost(const Node &n, const ApaRsPath_t &path) const {
    double cost = n.g, previousK = n.previousK;
    int gear = n.gear;
    for (int i = 0; i < APA_RS_MAX_SEGMENTS; ++i) {
      const double distance = path.Lengths[i];
      if (std::abs(distance) < 1.e-6)
        continue;
      const double k = path.Type[i] == APA_RS_SEG_LEFT
                           ? curvature
                           : path.Type[i] == APA_RS_SEG_RIGHT ? -curvature : 0;
      cost += increment(distance, k, gear, previousK);
      gear = distance > 0 ? 1 : -1;
      previousK = k;
    }
    return cost;
  }
  ApaParkingQuality_t rsQuality(const Node &n, const ApaRsPath_t &path) const {
    auto quality=n.quality;
    for (int i=0;i<APA_RS_MAX_SEGMENTS;++i)
      apaParkingRunQualityAdvance(&quality,path.Lengths[i]);
    return apaParkingRunQualityFinish(&quality,rsCost(n,path));
  }
  bool bounded(const Node &n, double h) const {

    return apaParkingQualityCompare({n.quality.Changes,n.quality.ShortRuns,n.g+h},bestQuality)>=0;
  }
  void evaluateCandidates(bool flush) {
    if (!planner.EvaluateCandidates) return;
    const auto began = std::chrono::steady_clock::now();
    const auto feedback = planner.EvaluateCandidates(flush, planner.CandidateUserData);
    executionSeconds += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - began).count();
    bestCost = feedback.BestCost;
    bestQuality={int(feedback.BestDirectionChanges),int(feedback.BestShortRuns),bestCost};
    executionBudgetFull = feedback.BudgetExhausted;
    executionChecks = feedback.EvaluatedCandidates;
    planner.HasAcceptedCandidate = std::isfinite(bestCost);
    if (planner.HasAcceptedCandidate) markSolution();
  }
  void markSolution() {
    markFirstExecutable();

    if (phaseFirstSolution<0 && apaParkingQualityCompare(bestQuality,phaseIncumbent)<0)
      phaseFirstSolution=expanded;
  }
  bool deliver(const std::vector<Control> &controls, ApaParkingQuality_t rank, int root) {
    if (controls.empty())
      return false;
    if (controls.size()>APA_PATH_SEG_MAX) {++capacityRejected;return false;}
    ApaHybridAStarMotionPath_t motion{};
    apaPathInit(&motion.Path);
    motion.Path.ValidFlag = true;
    motion.Path.PathType = APA_PATH_TYPE_ALL;
    motion.Path.PathSource = expanded == 0 ? APA_PATH_SOURCE_GEO : APA_PATH_SOURCE_SEARCH;
    Pose p = roots[root];
    for (size_t i = 0; i <= controls.size(); ++i) {
      const Pose world = toWorld(p);
      motion.Path.X[i] = world.x;
      motion.Path.Y[i] = world.y;
      motion.Path.Tht[i] = world.yaw;
      if (i < controls.size()) {
        motion.Path.OdoStep[i] = controls[i].distance;
        motion.Curvature[i] = controls[i].curvature;
        p = advance(p, controls[i].distance, controls[i].curvature);
      }
    }
    motion.Path.PathN = controls.size() + 1;
    motion.Path.SegNum = apaPathCalcSegNum(&motion.Path);
    motion.Path.SegNumFlag = true;
    ApaPath_t quality = motion.Path;
    if (!planner.CandidateCheck ||
        !planner.CandidateCheck(&motion, &quality, planner.CandidateUserData)) {
      ++planner.RejectedGoalCandidates;
      return false;
    }
    ++planner.AcceptedCandidates;
    if (!planner.EvaluateCandidates) {
      bestCost = rank.Cost;
      bestQuality = rank;
      planner.HasAcceptedCandidate = true;
      planner.BestQualityPath = quality;
      if (planner.CandidateSelected)
        planner.CandidateSelected(planner.CandidateUserData);
      markSolution();
    }
    std::fprintf(
        stderr,
        "[APA_STRUCTURED_CANDIDATE] expanded=%d cost=%.3f controls=%zu\n",
        expanded, rank.Cost, controls.size());
    return true;
  }
  bool accept(int index, const ApaRsPath_t &rsPath, ApaParkingQuality_t rank) {
    std::vector<Control> reverse, controls=rootPrefixes[nodes[index].root];
    for (int i=index;nodes[i].parent>=0;i=nodes[i].parent)
      append(reverse,nodes[i].distance,nodes[i].steer*curvature/2);
    for (auto it=reverse.rbegin();it!=reverse.rend();++it)
      append(controls,it->distance,it->curvature);
    for (auto c:apa::parking::controls(rsPath,radius)) append(controls,c.distance,c.curvature);
    if (controls.size()<=APA_PATH_SEG_MAX) return deliver(controls,rank,nodes[index].root);
    ++oversizedConnections;

    double length=0;for (auto c:controls) length+=std::abs(c.distance);
    CompletedRoute candidate{std::move(controls),length,nodes[index].root};
    const auto less=[](const CompletedRoute &a,const CompletedRoute &b) {
      if (a.controls.size()!=b.controls.size()) return a.controls.size()<b.controls.size();
      return a.length!=b.length ? a.length<b.length : a.root<b.root;
    };
    const auto at=std::lower_bound(overflowCandidates.begin(),overflowCandidates.end(),candidate,less);
    if (at!=overflowCandidates.end() && !less(candidate,*at) && !less(*at,candidate)) {++capacityPoolDropped;return false;}
    if (at==overflowCandidates.end() && overflowCandidates.size()>=3) {++capacityPoolDropped;return false;}
    overflowCandidates.insert(at,std::move(candidate));
    if (overflowCandidates.size()>3) {overflowCandidates.pop_back();++capacityPoolDropped;}
    return false;
  }
  ApaParkingQuality_t routeRank(const std::vector<Control> &controls, int root) {
    auto runs=ApaParkingRunQuality_t{};
    double cost=0,previousK=0;int gear=0;Pose p=roots[root];
    for (auto c:controls) {
      cost+=increment(c.distance,c.curvature,gear,previousK)+clearanceCost(p,c.distance,c.curvature);
      apaParkingRunQualityAdvance(&runs,c.distance);
      p=advance(p,c.distance,c.curvature);previousK=c.curvature;gear=c.distance>0 ? 1 : -1;
    }
    return apaParkingRunQualityFinish(&runs,cost);
  }
  void admitOverflow() {
    for (auto &candidate:overflowCandidates) {
      const int remaining=kNormalizationTrialLimit-int(normalizationTrials);
      if (remaining<=0) {++capacityRejected;continue;}
      auto &controls=candidate.controls;
      const auto normalized=apa::parking::normalize(controls,roots[candidate.root],radius,APA_PATH_SEG_MAX,
          [this](Pose p,double d,double k){return arc(p,d,k);},remaining);
      normalizationTrials+=normalized.trials;
      if (!normalized.fits) {++capacityRejected;continue;}
      ++normalizedCandidates;
      auto rank=routeRank(controls,candidate.root);
      if (apaParkingQualityCompare(rank,bestQuality)<0) {
        deliver(controls,rank,candidate.root);
        evaluateCandidates(true);
      }
    }
    overflowCandidates.clear();
  }
  void connect(int index, RsCollection &collection) {
    const Node node = nodes[index];
    std::sort(collection.paths.begin(), collection.paths.end(),
              [&](const auto &a, const auto &b) {
                return apaParkingQualityCompare(rsQuality(node,a),rsQuality(node,b))<0;
              });
    for (const auto &path : collection.paths) {
      auto quality = rsQuality(node,path);
      if (apaParkingQualityCompare(quality,bestQuality)>=0)
        break;
      if (path.TotalLength > std::max(8.0, 2 * node.h)) {
        ++connectionLengthRejected; continue;
      }
      ++connections;
      Pose p = node.pose;
      bool safe = true;
      for (int i = 0; i < APA_RS_MAX_SEGMENTS && safe; ++i) {
        const double distance = path.Lengths[i];
        if (std::abs(distance) < 1.e-6)
          continue;
        const double k =
            path.Type[i] == APA_RS_SEG_LEFT
                ? curvature
                : path.Type[i] == APA_RS_SEG_RIGHT ? -curvature : 0;
        safe = arc(p, distance, k);
        if (safe) quality.Cost += clearanceCost(p, distance, k);
        p = advance(p, distance, k);
      }
      if (!safe) ++connectionBlocked;

      if (safe && apaParkingQualityCompare(quality,bestQuality)<0 && std::hypot(p.x - goal.x, p.y - goal.y) < .005 &&
          std::abs(wrap(p.yaw - goal.yaw)) < .001)
        accept(index, path, quality);
    }

    evaluateCandidates(expanded == 0 || (phaseFirstSolution<0 && expanded%8==0));
  }
  static bool dominates(const Node &a,const Node &b) {

    const int possibleExtra=a.quality.OpenLength<b.quality.OpenLength &&
        apaPathDirectionRunIsShort(Real(a.quality.OpenLength));
    return a.g<=b.g+1.e-7 && a.controls<=b.controls &&
        a.quality.Changes<=b.quality.Changes &&
        a.quality.ShortRuns+possibleExtra<=b.quality.ShortRuns;
  }
  void push(Node n) {
    const Key state = key(n);
    auto &frontier = labels[state];
    bool guide=true, quality=true;
    for (const auto &label : frontier) {
      const auto &old = nodes[label.node];
      if (label.guide && old.guideG<=n.guideG+1.e-7 && old.controls<=n.controls) guide=false;
      if (label.quality && dominates(old,n)) quality=false;
    }
    if (!guide && !quality) { ++dominated; return; }

    if (!guide && qualityOnlyLabels>=kLabelLimit/4) {
      ++qualityCapacityRejected; return;
    }
    const size_t labelLimit = resolution==Resolution::Nominal ?
        kNominalLabelLimit : kLabelLimit;
    if (retiredLabels+nodes.size() >= labelLimit) {
      memoryFull = true;
      return;
    }
    frontier.erase(std::remove_if(frontier.begin(), frontier.end(),
                                  [&](Label &label) {
                                    const auto &old = nodes[label.node];
                                    const bool guided = guide &&
                                        old.guideG>=n.guideG-1.e-7 && old.controls>=n.controls;
                                    const bool qualified=quality && dominates(n,old);
                                    if (guided) label.guide=false;
                                    if (qualified) label.quality=false;
                                    const bool removed=!label.guide && !label.quality;
                                    if (removed && label.closed)
                                      ++reopened;
                                    return removed;
                                  }),
                   frontier.end());
    const int index = nodes.size();
    nodes.push_back(n);
    frontier.push_back({index,false,guide,quality});
    if (guide) open.push({n.guideG+kHeuristicWeight*n.h,index});
    else ++qualityOnlyLabels;
    if (quality) qualityOpen.push({n.g+kHeuristicWeight*n.h,index});
  }
  void expand(int index) {
    const Node n = nodes[index];
    auto guide=environment;
    const Pose world=toWorld(n.pose);
    guide.VehPosX=world.x;guide.VehPosY=world.y;guide.VehPosTheta=world.yaw;
    const bool deep=apaPlanningSlotDepthWeight(&guide)>.5;
    const std::array<int,5> straightFirst{0,-1,1,-2,2};
    for (int gear : {-1, 1})
      for (int choice=0;choice<5;++choice) {

        const int steer=deep ? straightFirst[choice] : choice-2;
        const int controls = n.controls + (n.gear != gear || n.steer != steer);
        if (controls > kSearchControlLimit) {++controlPruned;continue;}
        const double k = steer * curvature / 2;
        Pose previous = n.pose;
        double checked = 0, clearance = 0, guideClearance = 0;
        const int begin=resolution==Resolution::Nominal ? kNominalLengthIndex :
            resolution!=Resolution::Fine ? 2 : 0;
        const int end=resolution==Resolution::Nominal ? begin+1 : int(kLengths.size());
        for (int step=begin;step<end;++step) {
          if (resolution==Resolution::Fine && step==2) continue;
          const double length=kLengths[step];
          if (!arc(previous, gear * (length - checked), k))
            break;
          double guideIncrement=0;
          clearance += clearanceCost(previous, gear * (length - checked), k,&guideIncrement);
          guideClearance+=guideIncrement;
          previous = advance(n.pose, gear * length, k);
          checked = length;
          ++generated;
          const double g = n.g + clearance + increment(gear * length, k, n.gear,
                                                       n.previousK);
          const double h = heuristic(previous);
          Node next{previous,g,h,gear*length,index,gear,steer,controls};
          next.quality=n.quality;
          next.root=n.root;
          next.previousK=k;
          next.guideG=n.guideG+guideClearance+increment(gear*length,k,n.gear,n.previousK);
          apaParkingRunQualityAdvance(&next.quality,gear*length);
          if (bounded(next,h) || fieldIndex(previous)<0) continue;
          push(next);
        }
      }
  }

  bool needsDetail() const {

    return !planner.HasAcceptedCandidate || bestQuality.ShortRuns>0;
  }
  void seedStraightExit(const Node& root, int parent) {
    if (root.controls>=kSearchControlLimit) return;
    auto e=environment;
    const Pose origin=toWorld(root.pose);
    e.VehPosX=origin.x;e.VehPosY=origin.y;e.VehPosTheta=origin.yaw;
    const double exitDistance=apaPlanningSlotStraightExitDistance(&e);
    if (exitDistance==0) return;
    const int gear=exitDistance>0 ? 1 : -1;
    const double step=kLengths[kNominalLengthIndex];
    const double limit=std::min(8.0,std::abs(exitDistance)+step);
    Pose previous=root.pose;
    double cost=0,guideCost=0;
    int count=0;
    const int samples=int(std::floor((limit+1.e-6)/step));
    for (int sample=1;sample<=samples;++sample) {
      const double d=sample*step;
      if (!arc(previous,gear*step,0)) break;
      double guideIncrement=0;
      cost+=clearanceCost(previous,gear*step,0,&guideIncrement);
      guideCost+=guideIncrement;
      previous=advance(root.pose,gear*d,0);
      Node seed{previous,root.g+increment(gear*d,0,root.gear,root.previousK)+cost,
          heuristic(previous),gear*d,parent,gear,0,root.controls+1};
      seed.root=root.root;seed.quality=root.quality;
      seed.guideG=root.guideG+increment(gear*d,0,root.gear,root.previousK)+guideCost;
      apaParkingRunQualityAdvance(&seed.quality,gear*d);
      if (!bounded(seed,seed.h) && fieldIndex(previous)>=0) {
        const auto before=nodes.size();push(seed);count+=nodes.size()>before;
      }
    }

    if (count) std::fprintf(stderr,"[APA_SEARCH_STRAIGHT_EXIT] root=%d seeds=%d gear=%d step=%.2f\n",
        root.root,count,gear,step);
  }
  void configureClearance(bool preferred) {
    margins=minimumMargins;
    if (preferred)
      for (auto& margin:margins) margin=std::max(margin,APA_PLANNING_PREFERRED_CLEARANCE_M);
    corner=std::max({std::hypot(environment.VehLF+margins[0],environment.VehW/2+margins[2]),
        std::hypot(environment.VehLB+margins[1],environment.VehW/2+margins[2]),
        std::hypot(environment.VehWB+APA_PARKIN_PHYSICAL_MARGIN_M,environment.VehW/2+APA_PARKIN_PHYSICAL_MARGIN_M)});
  }
  void startGraph(Resolution next) {
    retiredLabels+=nodes.size();
    nodes.clear();
    labels.clear();
    open={}; qualityOpen={};
    memoryFull=false;
    resolution=next;

    configureClearance(next==Resolution::Nominal);
    phaseIncumbent=bestQuality;
    phaseFirstSolution=-1;
    improvementExpansions=0;

    if (!clear(goal)) return;

    const size_t begin=next==Resolution::Region ? 1 : next==Resolution::Tails ? anchorCount : 0;
    const size_t end=next==Resolution::Region ? anchorCount : next==Resolution::Tails ? roots.size() : 1;
    for (size_t i=begin;i<end;++i) {
      Pose pose=roots[i];
      if (!clear(pose)) continue;
      Node root{pose,0,0,0,-1,0,0,0};
      root.root=int(i);
      root.guideG=apaParkingGoalOffsetCost(std::hypot(pose.x,pose.y),wrap(pose.yaw),radius);
      for (auto c:rootPrefixes[i]) {
        double guideIncrement=0;
        const double movement=increment(c.distance,c.curvature,root.gear,root.previousK);
        root.g+=movement+clearanceCost(pose,c.distance,c.curvature,&guideIncrement);
        root.guideG+=movement+guideIncrement;
        apaParkingRunQualityAdvance(&root.quality,c.distance);
        pose=advance(pose,c.distance,c.curvature);
        root.gear=c.distance>0 ? 1 : -1;
        root.previousK=c.curvature;
        root.steer=3;
        ++root.controls;
      }
      root.pose=pose;root.h=heuristic(pose);
      const auto rootIndex=nodes.size();
      push(root);
      if (nodes.size()>rootIndex) seedStraightExit(root,int(rootIndex));
    }
  }
  void searchGraph() {
    const int beganExpanded=expanded;
    const int beganChecks=executionChecks;
    const int limit=resolution==Resolution::Nominal ? kNominalExpansionLimit :
        resolution==Resolution::Fine ? kExpandedLimit :
        std::min(kExpandedLimit,expanded+(resolution==Resolution::Tails ? 4000 : kDetailedExpansionLimit));
    reason="open_empty";
    while ((!open.empty() || !qualityOpen.empty()) && expanded<limit && !executionBudgetFull) {

      if (!planner.HasAcceptedCandidate && resolution!=Resolution::Fine &&
          executionChecks-beganChecks >= (resolution==Resolution::Nominal ? 3 : 4)) {
        reason="reserve_finer_admission";
        break;
      }

      const int qualityPeriod=phaseFirstSolution>=0 ? 4 : 8;
      const bool secondary=open.empty() ||
          (expanded%qualityPeriod==0 && !qualityOpen.empty());
      auto &queue=secondary ? qualityOpen : open;
      const auto entry=queue.top(); queue.pop();
      const int index=entry.index;
      auto it=labels.find(key(nodes[index]));
      if (it==labels.end()) continue;
      auto label=std::find_if(it->second.begin(),it->second.end(),
          [&](const Label &item){return item.node==index;});
      if (label==it->second.end() || label->closed ||
          !(secondary ? label->quality : label->guide)) continue;
      auto collection=rs(nodes[index].pose);
      if (!nodes[index].informed) {
        double h=std::numeric_limits<double>::infinity();
        for (const auto &path:collection.paths) h=std::min(h,double(path.TotalLength));
        nodes[index].informed=true;
        if (std::isfinite(h) && h>nodes[index].h+1.e-6) {
          nodes[index].h=h;
          queue.push({(secondary ? nodes[index].g : nodes[index].guideG)+kHeuristicWeight*h,index});
          continue;
        }
      }
      label->closed=true; ++expanded; qualityExpansions+=secondary;
      if (phaseFirstSolution>=0) {
        improvementExpansions=std::min(kQualityExpansionLimit,improvementExpansions+1);
        if (improvementExpansions>=kQualityExpansionLimit) {
          reason="quality_budget"; break;
        }
      }
      if (bounded(nodes[index],nodes[index].h)) continue;
      if (phaseFirstSolution<0 || expanded%8==0 || nodes[index].h<2.0)
        connect(index,collection);
      if (executionBudgetFull) break;
      if (!memoryFull) expand(index);
      if (expanded%5000==0)
        std::fprintf(stderr,"[APA_STRUCTURED_PROGRESS] expanded=%d labels=%zu best=%.3f\n",
            expanded,retiredLabels+nodes.size(),bestCost);
    }

    admitOverflow();
    evaluateCandidates(true);
    if (expanded>=limit) reason=limit<kExpandedLimit ? "resolution_budget" : "expansion_budget";
    else if (memoryFull) reason="label_capacity";
    std::fprintf(stderr,"[APA_SEARCH_RESOLUTION] mode=%s expanded=%d total_expanded=%d "
        "labels=%zu total_labels=%zu accepted=%d short_runs=%d reason=%s\n",
        resolution==Resolution::Nominal ? "nominal" : resolution==Resolution::Detailed ? "detailed" :
        resolution==Resolution::Region ? "region" : resolution==Resolution::Tails ? "tails" : "fine",
        expanded-beganExpanded,expanded,
        nodes.size(),retiredLabels+nodes.size(),int(planner.HasAcceptedCandidate),bestQuality.ShortRuns,reason);
  }

public:
  Search(ApaHybridAStar_t &p, const VehicleCollisionCheckInput_t &e,
         const ApaPlanningSceneQuery_t &s)
      : planner(p), environment(e), scene(s),
        radius(p.PlannerGlobal->Planner.Veh.MinCircle),
        curvature(1 / radius), worldOrigin{p.StartPose[0], p.StartPose[1],
                                           p.StartPose[2]},
        frameCos(std::cos(worldOrigin.yaw)),
        frameSin(std::sin(worldOrigin.yaw)), start{0, 0, 0},
        goal{frameCos * (p.GoalPose[0] - worldOrigin.x) +
                 frameSin * (p.GoalPose[1] - worldOrigin.y),
             -frameSin * (p.GoalPose[0] - worldOrigin.x) +
                 frameCos * (p.GoalPose[1] - worldOrigin.y),
             wrap(p.GoalPose[2] - worldOrigin.yaw)} {

    vehicleCollisionCheckResolveBodyMargins(&e, margins.data());
    minimumMargins=margins;
    configureClearance(false);
  }
  int run(bool directFirst = false) {
    const auto began = std::chrono::steady_clock::now();
    searchBegan=began;
    std::fprintf(stderr,
                 "[APA_STRUCTURED_BUDGET] expansions=%d labels=%d xy=%.3f "
                 "yaw_deg=%.3f heuristic_weight=%.2f nominal_step=%.2f "
                 "nominal_expansions=%d nominal_labels=%d\n",
                 kExpandedLimit, kLabelLimit, kXy, kYaw * 180 / kPi,
                 kHeuristicWeight,kLengths[kNominalLengthIndex],
                 kNominalExpansionLimit,kNominalLabelLimit);
    planner.HasAcceptedCandidate = false;
    planner.AcceptedCandidates = planner.RejectedGoalCandidates = 0;
    evaluateCandidates(true);

    roots.push_back(start);
    rootPrefixes.emplace_back();
    const double tolerance=planner.StartPositionTolerance;
    if (std::isfinite(tolerance) && tolerance>0) {
      const double offset=std::min(.15,.8*tolerance);
      for (double scale : {-.5,.5,-1.0,1.0}) {
        Pose anchor{scale*offset,0,0};
        if (clear(anchor)) {roots.push_back(anchor);rootPrefixes.emplace_back();}
      }
    }
    std::fprintf(stderr,"[APA_SEARCH_GOAL_REGION] anchors=%zu position_tolerance=%.6f\n",
        roots.size(),tolerance);
    anchorCount=roots.size();
    if (!clear(goal)) {
      reason = "endpoint_collision";
      return finish(began);
    }
    if (directFirst && clear(start)) {
      nodes.push_back({roots[0],0,heuristic(roots[0]),0,-1,0,0,0});
      auto collection=rs(roots[0]);
      connect(0,collection);
      nodes.clear();
    }
    if (executionBudgetFull) return finish(began);
    buildField();
    nodes.reserve(kLabelLimit);
    labels.reserve(kLabelLimit);
    startGraph(Resolution::Nominal);
    searchGraph();
    if (needsDetail() && !executionBudgetFull && expanded<kExpandedLimit &&
        retiredLabels+nodes.size()<kLabelLimit) {
      startGraph(Resolution::Detailed);
      searchGraph();
    }

    configureClearance(false);
    if (planner.BuildParkingSeed && (!planner.HasAcceptedCandidate || bestQuality.Changes>2) &&
        !executionBudgetFull && expanded<kExpandedLimit) {
      for (int i=0;i<5;++i) {
        ApaHybridAStarMotionPath_t seed{};
        if (!planner.BuildParkingSeed(i,&seed,planner.CandidateUserData) ||
            seed.Path.PathN<2 || seed.Path.PathN>APA_PATH_MAX_POINTS) continue;
        Pose anchor{frameCos*(seed.Path.X[0]-worldOrigin.x)+frameSin*(seed.Path.Y[0]-worldOrigin.y),
            -frameSin*(seed.Path.X[0]-worldOrigin.x)+frameCos*(seed.Path.Y[0]-worldOrigin.y),
            wrap(seed.Path.Tht[0]-worldOrigin.yaw)};
        if (std::hypot(anchor.x,anchor.y)>planner.StartPositionTolerance ||
            std::abs(anchor.yaw)>.001 || !clear(anchor)) continue;
        Pose pose=anchor;std::vector<Control> prefix;bool safe=true;
        for (int j=0;j+1<seed.Path.PathN && safe;++j) {
          const double d=seed.Path.OdoStep[j],k=seed.Curvature[j];
          safe=std::isfinite(d) && std::isfinite(k) && std::abs(k)<=curvature+1.e-5 && arc(pose,d,k);
          if (safe) {append(prefix,d,k);pose=advance(pose,d,k);}
        }
        if (!safe || prefix.empty() || prefix.size()>kSearchControlLimit) continue;
        roots.push_back(anchor);rootPrefixes.push_back(std::move(prefix));
      }
      std::fprintf(stderr,"[APA_SEARCH_TAIL_SEEDS] admitted=%zu\n",roots.size()-anchorCount);
      if (roots.size()>anchorCount && retiredLabels+nodes.size()<kLabelLimit) {
        startGraph(Resolution::Tails);searchGraph();
      }
    }
    if (needsDetail() && anchorCount>1 && !executionBudgetFull && expanded<kExpandedLimit &&
        retiredLabels+nodes.size()<kLabelLimit) {
      startGraph(Resolution::Region);searchGraph();
    }

    if ((!planner.HasAcceptedCandidate || (bestQuality.Changes>1 && bestQuality.ShortRuns>0)) &&
        !executionBudgetFull && expanded<kExpandedLimit &&
        retiredLabels+nodes.size()<kLabelLimit) {
      startGraph(Resolution::Fine);
      searchGraph();
    }
    return finish(began);
  }
  int finish(std::chrono::steady_clock::time_point began) {
    admitOverflow();
    evaluateCandidates(true);
    std::fprintf(stderr,"[APA_SEARCH_ADMISSION] rs_sweep_rejected=%zu rs_length_rejected=%zu "
        "capacity_rejected=%zu normalization_trials=%zu normalized_candidates=%zu control_pruned=%zu "
        "oversized_connections=%zu capacity_pool_dropped=%zu\n",
        connectionBlocked,connectionLengthRejected,capacityRejected,normalizationTrials,
        normalizedCandidates,controlPruned,oversizedConnections,capacityPoolDropped);
    if (executionBudgetFull) reason = "execution_budget";
    planner.ExpandIterations = expanded;
    planner.MotionCandidates = generated;
    planner.ReopenedStates = reopened;
    planner.GoalReached = planner.HasAcceptedCandidate;
    planner.RunState = planner.GoalReached ? APA_HYBRID_STATE_GOAL_REACHED
                                           : APA_HYBRID_STATE_FAILED;
    planner.Fault = planner.GoalReached
                        ? APA_HYBRID_FAULT_NO_ERR
                        : executionBudgetFull || expanded >= kExpandedLimit
                              ? APA_HYBRID_FAULT_EXCEED_CAL_POWER
                              : memoryFull
                                    ? APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT
                                    : APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY;
    if (planner.Diagnostics) {
      planner.Diagnostics->SearchFault = planner.Fault;
      planner.Diagnostics->AcceptedCandidates += planner.AcceptedCandidates;
      planner.Diagnostics->RejectedCandidates += planner.RejectedGoalCandidates;
      if (!planner.GoalReached) {
        const bool budget = executionBudgetFull || expanded >= kExpandedLimit || memoryFull;
        apaPlanningRecordFailure(planner.Diagnostics,apaPlanningSearchError(planner.Fault),
            reason,budget ? 7 : 1);
      }
    }
    std::fprintf(
        stderr,
        "[APA_STRUCTURED_SEARCH] success=%d reason=%s expanded=%d labels=%zu "
        "generated=%d dominated=%d reopened=%d collision_queries=%zu "
        "connections=%zu accepted=%ld rejected=%ld first_executable=%d "
        "execution_sec=%.6f elapsed=%.6f best_changes=%d best_short=%d "
        "quality_expanded=%d quality_only_labels=%d quality_capacity_rejected=%d "
        "improvement_expanded=%d resolutions=%d\n",
        planner.GoalReached, reason, expanded, retiredLabels+nodes.size(), generated,
        dominated, reopened, checks, connections,
        (long)planner.AcceptedCandidates, (long)planner.RejectedGoalCandidates,
        firstSolution, executionSeconds,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count(),bestQuality.Changes,bestQuality.ShortRuns,
        qualityExpansions,qualityOnlyLabels,qualityCapacityRejected,
        improvementExpansions,int(resolution)+1);
    std::fprintf(stderr,
        "[APA_SEARCH_TIMING] first_executable_sec=%.6f feasibility_execution_sec=%.6f "
        "quality_sec=%.6f quality_execution_sec=%.6f\n",
        firstSolution>=0 ? firstSolutionSeconds : -1.0,
        firstSolution>=0 ? firstExecutionSeconds : executionSeconds,firstSolution>=0 ?
        std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()-firstSolutionSeconds : 0,
        firstSolution>=0 ? executionSeconds-firstExecutionSeconds : 0);
    return planner.RunState;
  }
};
}

extern "C" int
apaParkingSearchRun(ApaHybridAStar_t *planner,
                    const VehicleCollisionCheckInput_t *environment,
                    const ApaPlanningSceneQuery_t *scene) {
  try {
    return Search(*planner, *environment, *scene).run();
  } catch (const std::bad_alloc &) {
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT;
    planner->RunState = planner->HasAcceptedCandidate
                            ? APA_HYBRID_STATE_GOAL_REACHED
                            : APA_HYBRID_STATE_FAILED;
    return planner->RunState;
  }
}

extern "C" int
apaParkingSearchLocal(ApaHybridAStar_t *planner,
    const VehicleCollisionCheckInput_t *environment,
    const ApaPlanningSceneQuery_t *scene) {
  try {
    return Search(*planner, *environment, *scene).run(true);
  } catch (const std::bad_alloc &) {
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT;
    planner->RunState = planner->HasAcceptedCandidate
                            ? APA_HYBRID_STATE_GOAL_REACHED
                            : APA_HYBRID_STATE_FAILED;
    return planner->RunState;
  }
}

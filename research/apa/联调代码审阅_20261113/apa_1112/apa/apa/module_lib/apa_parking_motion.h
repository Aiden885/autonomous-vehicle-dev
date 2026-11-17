#ifndef APA_PARKING_MOTION_H
#define APA_PARKING_MOTION_H

// Dynamic search geometry; normalized before conversion to fixed C arrays.
extern "C" {
#include "apaHybridAStar.h"
#include "apaParkingSearch.h"
}
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace apa::parking {
constexpr double Pi = 3.14159265358979323846;
struct Pose { double x, y, yaw; };
struct Control { double distance, curvature; };
using Route = std::vector<Control>;
inline double wrap(double a) { return std::remainder(a, 2 * Pi); }
inline Pose advance(Pose p, double d, double k) {
  const double a=d*k;
  const double chord=std::abs(a)<1.e-9 ? d : 2*std::sin(a/2)/k;
  return {p.x+chord*std::cos(p.yaw+a/2), p.y+chord*std::sin(p.yaw+a/2), wrap(p.yaw+a)};
}
inline void append(Route &route, double d, double k) {
  if (std::abs(d)<1.e-6) return;
  if (!route.empty() && route.back().distance*d>0 &&
      std::abs(route.back().curvature-k)<1.e-8) route.back().distance+=d;
  else route.push_back({d,k});
}
struct RsCollection {
  std::vector<ApaRsPath_t> paths;
  static void collect(const ApaRsPath_t *p, void *data) {
    if (std::isfinite(p->TotalLength))
      static_cast<RsCollection *>(data)->paths.push_back(*p);
  }
};
inline RsCollection connections(Pose from, Pose to, double radius) {
  const double c=std::cos(from.yaw),s=std::sin(from.yaw);
  Real relative[3]={Real(c*(to.x-from.x)+s*(to.y-from.y)),
                    Real(-s*(to.x-from.x)+c*(to.y-from.y)),Real(wrap(to.yaw-from.yaw))};
  RsCollection result;
  ApaRsVisitor_t visitor{&RsCollection::collect,&result};
  apaHybridAStarEnumerateRs(relative,radius,&visitor);
  return result;
}
inline Route controls(const ApaRsPath_t &path, double radius) {
  Route route;
  for (int i=0;i<APA_RS_MAX_SEGMENTS;++i)
    append(route,path.Lengths[i],path.Type[i]==APA_RS_SEG_LEFT ? 1/radius :
           path.Type[i]==APA_RS_SEG_RIGHT ? -1/radius : 0);
  return route;
}
inline ApaParkingQuality_t quality(const Route &route, double radius) {
  ApaParkingRunQuality_t runs{};
  double cost=0,previousK=0; int gear=0;
  for (auto c:route) {
    cost+=apaParkingMotionCost(c.distance,c.curvature,previousK,gear,1/radius);
    apaParkingRunQualityAdvance(&runs,c.distance);
    gear=c.distance>0 ? 1 : -1; previousK=c.curvature;
  }
  return apaParkingRunQualityFinish(&runs,cost);
}
struct Normalization {
  int trials=0, replacements=0;
  bool fits=false;
};
// Capacity repair preserves vertices and maneuver quality. Every integrated
// replacement needs a full swept check and scene-cost recomputation.
template<class Sweep>
Normalization normalize(Route &route, Pose origin, double radius, size_t capacity,
                        Sweep sweep, int trialLimit=256) {
  Normalization result;
  while (route.size()>capacity && result.trials<trialLimit) {
    std::vector<Pose> vertices{origin};
    for (auto c:route) vertices.push_back(advance(vertices.back(),c.distance,c.curvature));
    const auto before=quality(route,radius);
    bool replaced=false;
    for (size_t width=2;width<=route.size() && !replaced && result.trials<trialLimit;++width) {
      const size_t span=width;
      for (size_t i=0;i+span<=route.size() && !replaced && result.trials<trialLimit;++i) {
        const size_t j=i+span;
        auto options=connections(vertices[i],vertices[j],radius);
        std::sort(options.paths.begin(),options.paths.end(),[](const auto &a,const auto &b){return a.TotalLength<b.TotalLength;});
        for (const auto &rs:options.paths) {
          auto middle=controls(rs,radius);
          if (middle.size()>=span) continue;
          Route candidate;
          for (size_t n=0;n<i;++n) append(candidate,route[n].distance,route[n].curvature);
          for (auto c:middle) append(candidate,c.distance,c.curvature);
          for (size_t n=j;n<route.size();++n) append(candidate,route[n].distance,route[n].curvature);
          if (candidate.size()>=route.size() ||
              apaParkingQualityCompare(quality(candidate,radius),before)>0) continue;
          if (result.trials>=trialLimit) break;
          ++result.trials;
          Pose p=vertices[i]; bool safe=true;
          for (auto c:middle) {
            if (!sweep(p,c.distance,c.curvature)) {safe=false;break;}
            p=advance(p,c.distance,c.curvature);
          }
          if (!safe || std::hypot(p.x-vertices[j].x,p.y-vertices[j].y)>2.e-5 ||
              std::abs(wrap(p.yaw-vertices[j].yaw))>2.e-6) continue;
          p=origin;
          for (auto c:candidate) {
            if (!sweep(p,c.distance,c.curvature)) {safe=false;break;}
            p=advance(p,c.distance,c.curvature);
          }
          if (!safe || std::hypot(p.x-vertices.back().x,p.y-vertices.back().y)>5.e-5 ||
              std::abs(wrap(p.yaw-vertices.back().yaw))>5.e-6) continue;
          route=std::move(candidate); ++result.replacements; replaced=true; break;
        }
      }
    }
    if (!replaced) break;
  }
  result.fits=!route.empty() && route.size()<=capacity;
  return result;
}
}
#endif

#pragma once

#include "bunker_util/elevation_types.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

namespace bunker_elevation {

// -------------------------------------------------------
// ElevationMapDualLayer  (메인 알고리즘)
//   라벨 기반 low / high / vert 분리 → Kalman 업데이트.
//   labels: 0=수직, 1=ground수평, 2=upper수평, -1=unknown
//
//   레이어 구성:
//     elevation, elevation_var, surface_type  ← 최종 결과
//     elevation_low,  elevation_low_var,  surface_type_low
//     elevation_high, elevation_high_var, surface_type_high
//     ground_hits, high_hits              ← hit 카운트
// -------------------------------------------------------
class ElevationMapDualLayer
{
public:
  explicit ElevationMapDualLayer(const MapParams & params);

  void update(const std::vector<ElevPoint> & points,
              const std::vector<int> & labels);

  grid_map::GridMap & getMap() { return map_; }

private:
  // 셀별 low/high 업데이트 헬퍼
  void updateLowMapCell(const grid_map::Index & index,
                        const ElevPoint & pt);

  void updateHighMapCell(const grid_map::Index & index,
                         const ElevPoint & pt);

  void fuseLowHighToElevation(const grid_map::Index & index);

  // 클러스터 z 분산 계산 (셀 내 포인트 군)
  double computeVarZ(const std::vector<ElevPoint> & v) const;

  grid_map::GridMap map_;
  double default_meas_var_{0.01};
  int N_x_{0};
  int N_y_{0};
};

} // namespace bunker_elevation

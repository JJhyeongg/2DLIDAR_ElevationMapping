#pragma once

#include "bunker_util/elevation_types.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

namespace bunker_elevation {

// -------------------------------------------------------
// ElevationMapKalman
//   라벨 구분 없이 모든 포인트에 1D Kalman 필터 적용.
//   셀에 들어온 포인트를 순차적으로 Kalman 업데이트.
// -------------------------------------------------------
class ElevationMapKalman
{
public:
  explicit ElevationMapKalman(const MapParams & params);

  // 포인트 목록으로 맵 업데이트 (labels 무시)
  void update(const std::vector<ElevPoint> & points,
              const std::vector<int> & labels);

  grid_map::GridMap & getMap() { return map_; }

private:
  grid_map::GridMap map_;
  double default_meas_var_{0.01};
  int N_x_{0};
  int N_y_{0};
};

} // namespace bunker_elevation

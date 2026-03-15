#pragma once

#include "bunker_util/elevation_types.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

namespace bunker_elevation {

// -------------------------------------------------------
// ElevationMapMax
//   단순 max-z 방식: 셀에 들어온 포인트 중 가장 높은 z를 기록.
//   라벨을 사용하지 않음 (비교용 베이스라인).
// -------------------------------------------------------
class ElevationMapMax
{
public:
  explicit ElevationMapMax(const MapParams & params);

  // 포인트 목록으로 맵 업데이트 (labels 무시)
  void update(const std::vector<ElevPoint> & points,
              const std::vector<int> & labels);

  grid_map::GridMap & getMap() { return map_; }

private:
  grid_map::GridMap map_;
};

} // namespace bunker_elevation

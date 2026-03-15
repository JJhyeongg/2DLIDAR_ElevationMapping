#pragma once

#include <vector>
#include <string>
#include <limits>
#include <cmath>

namespace bunker_elevation {

// -------------------------------------------------------
// 포인트 하나 (LiDAR 스캔 → map 변환 결과)
// -------------------------------------------------------
struct ElevPoint {
  double x;     // rotated_map_frame 기준 x
  double y;     // rotated_map_frame 기준 y
  double z;     // rotated_map_frame 기준 z
  double r;     // LiDAR range
  double theta; // LiDAR angle (rad)
  double var_z; // z 공분산
  double x_l;   // LiDAR frame x (수평/수직 판별용)
  double y_l;   // LiDAR frame y (수평/수직 판별용)
};

// -------------------------------------------------------
// 지면 z 대역 추정 결과
// -------------------------------------------------------
struct GroundBand {
  bool   valid{false};
  double z_min{0.0};
  double z_max{0.0};
};

// -------------------------------------------------------
// 맵 생성 파라미터 (공통)
// -------------------------------------------------------
struct MapParams {
  double      resolution{0.1};
  double      width{50.0};
  double      height{50.0};
  std::string frame_id{"map_elev"};
  double      default_meas_var{0.01};
};

} // namespace bunker_elevation

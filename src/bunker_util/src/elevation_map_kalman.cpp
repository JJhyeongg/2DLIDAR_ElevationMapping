#include "bunker_util/elevation_map_kalman.hpp"
#include <unordered_map>
#include <limits>
#include <cmath>

namespace bunker_elevation {

ElevationMapKalman::ElevationMapKalman(const MapParams & params)
: default_meas_var_(params.default_meas_var)
{
  map_ = grid_map::GridMap({
      "elevation",
      "elevation_var",
      "surface_type"
  });
  map_.setFrameId(params.frame_id);

  grid_map::Length length(params.width, params.height);
  grid_map::Position center(0.0, 0.0);
  map_.setGeometry(length, params.resolution, center);

  for (auto & layer : map_.getLayers()) {
    map_[layer].setConstant(std::numeric_limits<float>::quiet_NaN());
  }

  const grid_map::Size & sz = map_.getSize();
  N_x_ = sz(0);
  N_y_ = sz(1);
}

void ElevationMapKalman::update(const std::vector<ElevPoint> & points,
                                const std::vector<int> & /* labels */)
{
  // 셀별로 포인트 모으기
  struct CellPts { std::vector<ElevPoint> pts; };
  std::unordered_map<long long, CellPts> cell_map;
  cell_map.reserve(points.size());

  for (const auto & pt : points) {
    grid_map::Position pos(pt.x, pt.y);
    if (!map_.isInside(pos)) continue;

    grid_map::Index index;
    map_.getIndex(pos, index);

    long long key = static_cast<long long>(index(0)) * 1000000LL
                  + static_cast<long long>(index(1));
    cell_map[key].pts.push_back(pt);
  }

  // 각 셀에 대해 순차 Kalman 업데이트
  for (const auto & kv : cell_map) {
    const auto & cp = kv.second;
    if (cp.pts.empty()) continue;

    int ix = static_cast<int>(kv.first / 1000000LL);
    int iy = static_cast<int>(kv.first % 1000000LL);
    if (ix < 0 || ix >= N_x_ || iy < 0 || iy >= N_y_) continue;

    grid_map::Index index(ix, iy);
    float & z_cell   = map_.at("elevation",     index);
    float & var_cell = map_.at("elevation_var", index);
    float & stype    = map_.at("surface_type",  index);

    size_t start = 0;

    // 셀이 미초기화 상태면 첫 포인트로 초기화
    if (!std::isfinite(z_cell) || !std::isfinite(var_cell)) {
      const auto & p0 = cp.pts[0];
      double v0 = (std::isfinite(p0.var_z) && p0.var_z > 1e-8)
                  ? p0.var_z : default_meas_var_;
      z_cell   = static_cast<float>(p0.z);
      var_cell = static_cast<float>(v0);
      stype    = 1.0f;
      start    = 1;
    }

    // 나머지 포인트 Kalman 업데이트
    for (size_t j = start; j < cp.pts.size(); ++j) {
      const auto & p = cp.pts[j];
      double z_m = p.z;
      double v_m = (std::isfinite(p.var_z) && p.var_z > 1e-8)
                   ? p.var_z : default_meas_var_;

      double mu_old  = static_cast<double>(z_cell);
      double var_old = static_cast<double>(var_cell);
      if (var_old < 1e-10) var_old = 1e-10;

      double v_new = (var_old * v_m) / (var_old + v_m);
      double m_new = (v_m * mu_old + var_old * z_m) / (var_old + v_m);
      if (v_new < 1e-8) v_new = 1e-8;

      z_cell   = static_cast<float>(m_new);
      var_cell = static_cast<float>(v_new);
    }

    stype = 1.0f;
  }
}

} // namespace bunker_elevation

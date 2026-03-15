#include "bunker_util/elevation_map_dual_layer.hpp"
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>

namespace bunker_elevation {

ElevationMapDualLayer::ElevationMapDualLayer(const MapParams & params)
: default_meas_var_(params.default_meas_var)
{
  map_ = grid_map::GridMap({
      "elevation",
      "elevation_var",
      "surface_type",
      "elevation_low",
      "elevation_low_var",
      "surface_type_low",
      "elevation_high",
      "elevation_high_var",
      "surface_type_high",
      "ground_hits",
      "high_hits"
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

// -------------------------------------------------------
// update: label 기반 low / high / vert 분리 후 업데이트
// -------------------------------------------------------
void ElevationMapDualLayer::update(const std::vector<ElevPoint> & points,
                                   const std::vector<int> & labels)
{
  struct CellPts {
    std::vector<ElevPoint> low_pts;   // label == 1 (ground 수평)
    std::vector<ElevPoint> high_pts;  // label == 2 (upper 수평)
    std::vector<ElevPoint> vert_pts;  // label == 0 (수직)
  };

  std::unordered_map<long long, CellPts> cell_map;
  cell_map.reserve(points.size());

  // --- 1) 셀별 low/high/vert 분리 ---
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & pt = points[i];
    int label = (i < labels.size()) ? labels[i] : -1;
    if (label < 0) continue;

    grid_map::Position pos(pt.x, pt.y);
    if (!map_.isInside(pos)) continue;

    grid_map::Index index;
    map_.getIndex(pos, index);

    long long key = static_cast<long long>(index(0)) * 1000000LL
                  + static_cast<long long>(index(1));

    auto & cm = cell_map[key];
    if      (label == 1) cm.low_pts.push_back(pt);
    else if (label == 2) cm.high_pts.push_back(pt);
    else if (label == 0) cm.vert_pts.push_back(pt);
  }

  // --- 2) 각 셀 처리 ---
  for (const auto & kv : cell_map) {
    const CellPts & cp = kv.second;

    int ix = static_cast<int>(kv.first / 1000000LL);
    int iy = static_cast<int>(kv.first % 1000000LL);
    if (ix < 0 || ix >= N_x_ || iy < 0 || iy >= N_y_) continue;

    grid_map::Index index(ix, iy);

    // hit 카운트 초기화 (NaN → 0)
    float & g_hits = map_.at("ground_hits", index);
    float & h_hits = map_.at("high_hits",   index);
    if (!std::isfinite(g_hits) || g_hits < 0.0f) g_hits = 0.0f;
    if (!std::isfinite(h_hits) || h_hits < 0.0f) h_hits = 0.0f;

    // ---------- low 후보 ----------
    bool      has_low = false;
    ElevPoint best_low;

    if (!cp.low_pts.empty()) {
      has_low  = true;
      best_low = *std::min_element(
          cp.low_pts.begin(), cp.low_pts.end(),
          [](const ElevPoint & a, const ElevPoint & b) { return a.z < b.z; });
      g_hits += static_cast<float>(cp.low_pts.size());
    }

    // ---------- high 후보 (수평upper + vertical 확장) ----------
    bool      has_high_base = !cp.high_pts.empty();
    ElevPoint best_high;
    double    var_high_cluster = default_meas_var_;

    if (has_high_base) {
      best_high = *std::max_element(
          cp.high_pts.begin(), cp.high_pts.end(),
          [](const ElevPoint & a, const ElevPoint & b) { return a.z < b.z; });
      var_high_cluster = computeVarZ(cp.high_pts);
    }

    bool      has_vert = !cp.vert_pts.empty();
    ElevPoint best_vert;
    double    var_vert_cluster = default_meas_var_;

    if (has_vert) {
      best_vert = *std::max_element(
          cp.vert_pts.begin(), cp.vert_pts.end(),
          [](const ElevPoint & a, const ElevPoint & b) { return a.z < b.z; });
      var_vert_cluster = computeVarZ(cp.vert_pts);
    }

    constexpr double MIN_OBS_HEIGHT = 0.06;  // 6cm 이상 차이 → 구조물
    constexpr double EPS_HIGH_EXT   = 0.02;  // vert가 high보다 2cm 이상 높으면 교체

    bool      has_high = false;
    ElevPoint final_high;
    double    var_high_final = default_meas_var_;

    if (has_high_base) {
      final_high     = best_high;
      has_high       = true;
      var_high_final = var_high_cluster;

      // vertical이 더 높으면 high를 올려줌
      if (has_vert && best_vert.z > final_high.z + EPS_HIGH_EXT) {
        final_high     = best_vert;
        var_high_final = var_vert_cluster;
      }
    } else {
      if (has_vert && has_low && best_vert.z >= best_low.z + MIN_OBS_HEIGHT) {
        final_high     = best_vert;
        has_high       = true;
        var_high_final = var_vert_cluster;
      } else if (has_vert && !has_low) {
        // 지면 없이 수직만 보이는 셀 → vertical을 high로
        final_high     = best_vert;
        has_high       = true;
        var_high_final = var_vert_cluster;
      }
    }

    if (has_high) {
      final_high.var_z = var_high_final;
      h_hits += static_cast<float>(cp.high_pts.size() + cp.vert_pts.size());
    }

    // ---------- 실제 레이어 업데이트 ----------
    if (has_low) {
      for (const auto & p_low : cp.low_pts) {
        updateLowMapCell(index, p_low);
      }
    }
    if (has_high) {
      updateHighMapCell(index, final_high);
    }

    fuseLowHighToElevation(index);
  }
}

// -------------------------------------------------------
// low 레이어: 1D Kalman (양방향 허용, 지면 추적)
// -------------------------------------------------------
void ElevationMapDualLayer::updateLowMapCell(const grid_map::Index & index,
                                             const ElevPoint & pt)
{
  float & z_low    = map_.at("elevation_low",     index);
  float & var_low  = map_.at("elevation_low_var", index);
  float & type_low = map_.at("surface_type_low",  index);

  double z_meas   = pt.z;
  double var_meas = (std::isfinite(pt.var_z) && pt.var_z > 1e-10)
                    ? pt.var_z : default_meas_var_;

  if (!std::isfinite(z_low) || !std::isfinite(var_low) || var_low <= 0.0f) {
    z_low    = static_cast<float>(z_meas);
    var_low  = static_cast<float>(var_meas);
    type_low = 1.0f;
    return;
  }

  double mu_old  = static_cast<double>(z_low);
  double var_old = static_cast<double>(var_low);
  if (var_old < 1e-10) var_old = 1e-10;

  double v_new = (var_old * var_meas) / (var_old + var_meas);
  double m_new = (var_meas * mu_old + var_old * z_meas) / (var_old + var_meas);
  if (v_new < 1e-8) v_new = 1e-8;

  z_low    = static_cast<float>(m_new);
  var_low  = static_cast<float>(v_new);
  type_low = 1.0f;
}

// -------------------------------------------------------
// high 레이어: 단조증가 업데이트 (장애물 영속성 보장)
// -------------------------------------------------------
void ElevationMapDualLayer::updateHighMapCell(const grid_map::Index & index,
                                              const ElevPoint & pt)
{
  float & z_high    = map_.at("elevation_high",     index);
  float & var_high  = map_.at("elevation_high_var", index);
  float & type_high = map_.at("surface_type_high",  index);

  double z_meas   = pt.z;
  double var_meas = (std::isfinite(pt.var_z) && pt.var_z > 1e-10)
                    ? pt.var_z : default_meas_var_;

  if (!std::isfinite(z_high) || !std::isfinite(var_high) || var_high <= 0.0f) {
    z_high    = static_cast<float>(z_meas);
    var_high  = static_cast<float>(var_meas);
    type_high = 2.0f;
    return;
  }

  // high는 절대 낮아지지 않음 (장애물 영속성)
  constexpr double EPS_Z = 1e-4;
  if (z_meas <= static_cast<double>(z_high) + EPS_Z) {
    return;
  }

  z_high    = static_cast<float>(z_meas);
  var_high  = static_cast<float>(var_meas);
  type_high = 2.0f;
}

// -------------------------------------------------------
// low + high → 최종 elevation 결정
// -------------------------------------------------------
void ElevationMapDualLayer::fuseLowHighToElevation(const grid_map::Index & index)
{
  float & z_final = map_.at("elevation",     index);
  float & v_final = map_.at("elevation_var", index);
  float & type    = map_.at("surface_type",  index);

  const float z_low  = map_.at("elevation_low",     index);
  const float v_low  = map_.at("elevation_low_var", index);
  const float z_high = map_.at("elevation_high",     index);
  const float v_high = map_.at("elevation_high_var", index);

  const float g_hits_raw = map_.at("ground_hits", index);
  const float h_hits_raw = map_.at("high_hits",   index);

  float g_hits = (std::isfinite(g_hits_raw) && g_hits_raw > 0.0f) ? g_hits_raw : 0.0f;
  float h_hits = (std::isfinite(h_hits_raw) && h_hits_raw > 0.0f) ? h_hits_raw : 0.0f;

  bool has_low  = std::isfinite(z_low)  && std::isfinite(v_low)  && v_low  > 0.0f;
  bool has_high = std::isfinite(z_high) && std::isfinite(v_high) && v_high > 0.0f;

  const auto nan = std::numeric_limits<float>::quiet_NaN();

  if (!has_low && !has_high) {
    z_final = nan; v_final = nan; type = -1.0f;
    return;
  }
  if (has_low && !has_high) {
    z_final = z_low; v_final = v_low; type = 1.0f;
    return;
  }
  if (!has_low && has_high) {
    z_final = z_high; v_final = v_high; type = 2.0f;
    return;
  }

  // 둘 다 있을 때: hit 카운트 기반 결정
  constexpr float MIN_HIGH_HITS  = 30.0f;
  constexpr float HIGH_DOM_RATIO = 1.0f;
  constexpr float MIN_OBS_HEIGHT = 0.1f;

  float dz = z_high - z_low;

  bool high_trustable =
      (h_hits >= MIN_HIGH_HITS) &&
      (h_hits >= HIGH_DOM_RATIO * g_hits) &&
      (dz >= MIN_OBS_HEIGHT);

  if (high_trustable) {
    z_final = z_high; v_final = v_high; type = 2.0f;
  } else {
    z_final = z_low; v_final = v_low; type = 1.0f;
  }
}

// -------------------------------------------------------
// 셀 내 z 클러스터 분산
// -------------------------------------------------------
double ElevationMapDualLayer::computeVarZ(const std::vector<ElevPoint> & v) const
{
  const size_t n = v.size();
  if (n < 2) return default_meas_var_;

  double mean = 0.0;
  for (const auto & p : v) mean += p.z;
  mean /= static_cast<double>(n);

  double s2 = 0.0;
  for (const auto & p : v) {
    double dz = p.z - mean;
    s2 += dz * dz;
  }
  s2 /= static_cast<double>(n - 1);

  return (std::isfinite(s2) && s2 > 1e-10) ? s2 : default_meas_var_;
}

} // namespace bunker_elevation

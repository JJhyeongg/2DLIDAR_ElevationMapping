#include "bunker_util/elevation_map_max.hpp"
#include <limits>
#include <cmath>

namespace bunker_elevation {

ElevationMapMax::ElevationMapMax(const MapParams & params)
{
  map_ = grid_map::GridMap({"elevation", "elevation_var"});
  map_.setFrameId(params.frame_id);

  grid_map::Length length(params.width, params.height);
  grid_map::Position center(0.0, 0.0);
  map_.setGeometry(length, params.resolution, center);

  map_["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
  map_["elevation_var"].setConstant(std::numeric_limits<float>::quiet_NaN());
}

void ElevationMapMax::update(const std::vector<ElevPoint> & points,
                             const std::vector<int> & /* labels */)
{
  for (const auto & pt : points) {
    grid_map::Position pos(pt.x, pt.y);
    if (!map_.isInside(pos)) {
      continue;
    }

    float & cell_elev = map_.atPosition("elevation",     pos);
    float & cell_var  = map_.atPosition("elevation_var", pos);

    const float z_meas   = static_cast<float>(pt.z);
    const float var_meas = static_cast<float>(pt.var_z);

    // 첫 관측
    if (!std::isfinite(cell_elev)) {
      cell_elev = z_meas;
      cell_var  = var_meas;
      continue;
    }

    // max-z 정책: 더 높은 값이 들어올 때만 교체
    if (z_meas > cell_elev) {
      cell_elev = z_meas;
      cell_var  = var_meas;
    }
  }
}

} // namespace bunker_elevation

#include "pointcloud_to_occupancy_grid/occupancy_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pointcloud_to_occupancy_grid {

OccupancyMap::OccupancyMap() : OccupancyMap(Options{}) {}

OccupancyMap::OccupancyMap(Options options) : options_(options) {
  grid_reso_ = options_.resolution * static_cast<float>(kSubGridWidth);
  ReleaseResources();
}

OccupancyMap::~OccupancyMap() { ReleaseResources(); }

bool OccupancyMap::EnsureBounds(float min_x, float min_y, float max_x, float max_y) {
  if (min_x > max_x || min_y > max_y) {
    return false;
  }

  const float snapped_min_x = SnapMin(min_x, grid_reso_);
  const float snapped_min_y = SnapMin(min_y, grid_reso_);
  const float snapped_max_x = SnapMax(max_x, grid_reso_);
  const float snapped_max_y = SnapMax(max_y, grid_reso_);

  if (grids_ == nullptr) {
    return Allocate(snapped_min_x, snapped_min_y, snapped_max_x, snapped_max_y);
  }

  const float target_min_x = std::min(min_x_, snapped_min_x);
  const float target_min_y = std::min(min_y_, snapped_min_y);
  const float target_max_x = std::max(max_x_, snapped_max_x);
  const float target_max_y = std::max(max_y_, snapped_max_y);

  if (target_min_x == min_x_ && target_min_y == min_y_ && target_max_x == max_x_ && target_max_y == max_y_) {
    return true;
  }

  // Pad by 20 subgrid cells in each expanding direction to amortize resize cost.
  const float pad = grid_reso_ * 20.0f;
  const float padded_min_x = target_min_x - (target_min_x < min_x_ ? pad : 0.0f);
  const float padded_min_y = target_min_y - (target_min_y < min_y_ ? pad : 0.0f);
  const float padded_max_x = target_max_x + (target_max_x > max_x_ ? pad : 0.0f);
  const float padded_max_y = target_max_y + (target_max_y > max_y_ ? pad : 0.0f);
  return Resize(padded_min_x, padded_min_y, padded_max_x, padded_max_y);
}

void OccupancyMap::SetHitPoint(float px, float py, bool hit, float height) {
  if (grids_ == nullptr) {
    return;
  }

  int x_index = 0;
  int y_index = 0;
  if (!GetDataIndex(px, py, x_index, y_index)) {
    return;
  }

  UpdateCell(Eigen::Vector2i(x_index, y_index), hit, height);
}

void OccupancyMap::SetMissPoint(float point_x, float point_y, float laser_origin_x, float laser_origin_y, float height,
                                float lidar_height) {
  if (grids_ == nullptr) {
    return;
  }

  int point_x_index = 0;
  int point_y_index = 0;
  int origin_x_index = 0;
  int origin_y_index = 0;
  if (!GetDataIndex(point_x, point_y, point_x_index, point_y_index) ||
      !GetDataIndex(laser_origin_x, laser_origin_y, origin_x_index, origin_y_index)) {
    return;
  }

  const int diff_x = point_x_index - origin_x_index;
  const int diff_y = point_y_index - origin_y_index;
  if (diff_x == 0 && diff_y == 0) {
    return;
  }

  if (std::abs(diff_y) > std::abs(diff_x)) {
    const float slope = static_cast<float>(diff_x) / static_cast<float>(diff_y);
    const float delta_height = (lidar_height - height) / static_cast<float>(diff_y);
    const int sign = diff_y > 0 ? 1 : -1;

    for (int j = sign; j != diff_y; j += sign) {
      const int i = static_cast<int>(std::round(static_cast<float>(j) * slope));
      UpdateCell(Eigen::Vector2i(origin_x_index + i, origin_y_index + j),
                 false, lidar_height - static_cast<float>(j) * delta_height);
    }
  } else {
    const float slope = static_cast<float>(diff_y) / static_cast<float>(diff_x);
    const float delta_height = (lidar_height - height) / static_cast<float>(diff_x);
    const int sign = diff_x > 0 ? 1 : -1;

    for (int i = sign; i != diff_x; i += sign) {
      const int j = static_cast<int>(std::round(static_cast<float>(i) * slope));
      UpdateCell(Eigen::Vector2i(origin_x_index + i, origin_y_index + j),
                 false, lidar_height - static_cast<float>(i) * delta_height);
    }
  }
}

bool OccupancyMap::GetDataIndex(float x, float y, int &x_index, int &y_index) const {
  if (grids_ == nullptr || x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_) {
    return false;
  }

  const int max_x_index = grid_size_x_ * kSubGridWidth - 1;
  const int max_y_index = grid_size_y_ * kSubGridWidth - 1;
  x_index = std::clamp(static_cast<int>(std::floor((x - min_x_) / options_.resolution)), 0, max_x_index);
  y_index = std::clamp(static_cast<int>(std::floor((y - min_y_) / options_.resolution)), 0, max_y_index);
  return true;
}

RasterMapData OccupancyMap::ToRasterMap() const {
  RasterMapData map;
  if (grids_ == nullptr) {
    return map;
  }

  map.width = static_cast<std::uint32_t>(grid_size_x_ * kSubGridWidth);
  map.height = static_cast<std::uint32_t>(grid_size_y_ * kSubGridWidth);
  map.resolution = options_.resolution;
  map.origin_x = min_x_;
  map.origin_y = min_y_;
  map.data.assign(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height), -1);

  const std::size_t grid_map_size = map.data.size();
  if (grid_map_size == 0) {
    return map;
  }

  for (int big_x = 0; big_x < grid_size_x_; ++big_x) {
    for (int big_y = 0; big_y < grid_size_y_; ++big_y) {
      if (grids_[big_x][big_y].IsEmpty()) {
        continue;
      }

      for (int sub_x = 0; sub_x < kSubGridWidth; ++sub_x) {
        for (int sub_y = 0; sub_y < kSubGridWidth; ++sub_y) {
          const int x = (big_x << SubGrid::kSubGridBits) + sub_x;
          const int y = (big_y << SubGrid::kSubGridBits) + sub_y;
          if (x < 0 || y < 0 || x >= static_cast<int>(map.width) || y >= static_cast<int>(map.height)) {
            continue;
          }

          const float log_odds = grids_[big_x][big_y].GetLogOdds(sub_x, sub_y);
          const unsigned int visit_count = grids_[big_x][big_y].GetVisitCount(sub_x, sub_y);
          if (visit_count < 4) {
            continue;
          }

          const std::size_t index = MapIndex(map.width, x, y);
          if (log_odds > options_.occupancy_threshold) {
            map.data[index] = 100;
            continue;
          }

          const int min_neighbor_x = std::max(0, x - 1);
          const int max_neighbor_x = std::min(static_cast<int>(map.width) - 1, x + 1);
          const int min_neighbor_y = std::max(0, y - 1);
          const int max_neighbor_y = std::min(static_cast<int>(map.height) - 1, y + 1);
          for (int neighbor_x = min_neighbor_x; neighbor_x <= max_neighbor_x; ++neighbor_x) {
            for (int neighbor_y = min_neighbor_y; neighbor_y <= max_neighbor_y; ++neighbor_y) {
              const std::size_t neighbor_index = MapIndex(map.width, neighbor_x, neighbor_y);
              if (map.data[neighbor_index] < 0) {
                map.data[neighbor_index] = 0;
              }
            }
          }
        }
      }
    }
  }

  return map;
}

bool OccupancyMap::IsEmpty() const { return grids_ == nullptr; }

void OccupancyMap::GetMinAndMax(float &min_x, float &min_y, float &max_x, float &max_y) const {
  min_x = min_x_;
  min_y = min_y_;
  max_x = max_x_;
  max_y = max_y_;
}

std::size_t OccupancyMap::MapIndex(std::size_t width, int x, int y) {
  return static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
}

bool OccupancyMap::Allocate(float min_x, float min_y, float max_x, float max_y) {
  ReleaseResources();
  min_x_ = min_x;
  min_y_ = min_y;
  max_x_ = max_x;
  max_y_ = max_y;

  grid_size_x_ = static_cast<int>(std::ceil((max_x_ - min_x_) / grid_reso_)) + 1;
  grid_size_y_ = static_cast<int>(std::ceil((max_y_ - min_y_) / grid_reso_)) + 1;
  if (grid_size_x_ <= 0 || grid_size_y_ <= 0) {
    ReleaseResources();
    return false;
  }

  grids_ = new SubGrid *[grid_size_x_];
  for (int x = 0; x < grid_size_x_; ++x) {
    grids_[x] = new SubGrid[grid_size_y_];
  }
  return true;
}

bool OccupancyMap::Resize(float min_x, float min_y, float max_x, float max_y) {
  if (grids_ == nullptr) {
    return Allocate(min_x, min_y, max_x, max_y);
  }

  const int new_grid_size_x = static_cast<int>(std::ceil((max_x - min_x) / grid_reso_)) + 1;
  const int new_grid_size_y = static_cast<int>(std::ceil((max_y - min_y) / grid_reso_)) + 1;
  if (new_grid_size_x <= 0 || new_grid_size_y <= 0) {
    return false;
  }

  auto **new_grids = new SubGrid *[new_grid_size_x];
  for (int x = 0; x < new_grid_size_x; ++x) {
    new_grids[x] = new SubGrid[new_grid_size_y];
  }

  const int min_grid_x = static_cast<int>(std::round((min_x - min_x_) / grid_reso_));
  const int min_grid_y = static_cast<int>(std::round((min_y - min_y_) / grid_reso_));
  const int dx = std::max(0, min_grid_x);
  const int dy = std::max(0, min_grid_y);
  const int dx_max = std::min(grid_size_x_, min_grid_x + new_grid_size_x);
  const int dy_max = std::min(grid_size_y_, min_grid_y + new_grid_size_y);

  for (int x = dx; x < dx_max; ++x) {
    for (int y = dy; y < dy_max; ++y) {
      new_grids[x - min_grid_x][y - min_grid_y] = std::move(grids_[x][y]);
    }
  }

  for (int x = 0; x < grid_size_x_; ++x) {
    delete[] grids_[x];
  }
  delete[] grids_;

  grids_ = new_grids;
  min_x_ = min_x;
  min_y_ = min_y;
  max_x_ = max_x;
  max_y_ = max_y;
  grid_size_x_ = new_grid_size_x;
  grid_size_y_ = new_grid_size_y;
  return true;
}

void OccupancyMap::ReleaseResources() {
  if (grids_ != nullptr) {
    for (int x = 0; x < grid_size_x_; ++x) {
      delete[] grids_[x];
    }
    delete[] grids_;
    grids_ = nullptr;
  }

  min_x_ = std::numeric_limits<float>::max();
  min_y_ = std::numeric_limits<float>::max();
  max_x_ = std::numeric_limits<float>::lowest();
  max_y_ = std::numeric_limits<float>::lowest();
  grid_size_x_ = 0;
  grid_size_y_ = 0;
}

void OccupancyMap::UpdateCell(const Eigen::Vector2i &point_index, bool hit, float height) {
  if (grids_ == nullptr) {
    return;
  }

  const int x_index = point_index.x();
  const int y_index = point_index.y();
  const int grid_x = x_index >> SubGrid::kSubGridBits;
  const int grid_y = y_index >> SubGrid::kSubGridBits;
  if (grid_x < 0 || grid_x >= grid_size_x_ || grid_y < 0 || grid_y >= grid_size_y_) {
    return;
  }

  const int sub_x = x_index - (grid_x << SubGrid::kSubGridBits);
  const int sub_y = y_index - (grid_y << SubGrid::kSubGridBits);
  if (sub_x < 0 || sub_x >= kSubGridWidth || sub_y < 0 || sub_y >= kSubGridWidth) {
    return;
  }

  grids_[grid_x][grid_y].SetGridHitPoint(hit, sub_x, sub_y, height, options_.log_odds_params);
}

float OccupancyMap::SnapMin(float value, float resolution) {
  return static_cast<float>(std::floor(value / resolution)) * resolution;
}

float OccupancyMap::SnapMax(float value, float resolution) {
  return static_cast<float>(std::ceil(value / resolution)) * resolution;
}

}  // namespace pointcloud_to_occupancy_grid

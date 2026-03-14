#pragma once

#include "pointcloud_to_occupancy_grid/subgrid.hpp"
#include "pointcloud_to_occupancy_grid/types.hpp"

#include <Eigen/Core>

#include <cstddef>

namespace pointcloud_to_occupancy_grid {

class OccupancyMap {
 public:
  struct Options {
    float resolution = 0.1f;
    float occupancy_threshold = 0.0f;
    LogOddsParams log_odds_params;
  };

  OccupancyMap();
  explicit OccupancyMap(Options options);
  ~OccupancyMap();

  OccupancyMap(const OccupancyMap &) = delete;
  OccupancyMap &operator=(const OccupancyMap &) = delete;

  bool EnsureBounds(float min_x, float min_y, float max_x, float max_y);
  void SetHitPoint(float px, float py, bool hit, float height);
  void SetMissPoint(float point_x, float point_y, float laser_origin_x, float laser_origin_y, float height,
                    float lidar_height);
  bool GetDataIndex(float x, float y, int &x_index, int &y_index) const;
  RasterMapData ToRasterMap() const;
  bool IsEmpty() const;
  void GetMinAndMax(float &min_x, float &min_y, float &max_x, float &max_y) const;
  float GetResolution() const { return options_.resolution; }

 private:
  static std::size_t MapIndex(std::size_t width, int x, int y);

  bool Allocate(float min_x, float min_y, float max_x, float max_y);
  bool Resize(float min_x, float min_y, float max_x, float max_y);
  void ReleaseResources();
  void UpdateCell(const Eigen::Vector2i &point_index, bool hit, float height);
  static float SnapMin(float value, float resolution);
  static float SnapMax(float value, float resolution);

  static constexpr int kSubGridWidth = SubGrid::kWidth;

  float grid_reso_ = 0.0f;
  float min_x_ = 0.0f;
  float min_y_ = 0.0f;
  float max_x_ = 0.0f;
  float max_y_ = 0.0f;
  int grid_size_x_ = 0;
  int grid_size_y_ = 0;

  Options options_;
  SubGrid **grids_ = nullptr;
};

}  // namespace pointcloud_to_occupancy_grid

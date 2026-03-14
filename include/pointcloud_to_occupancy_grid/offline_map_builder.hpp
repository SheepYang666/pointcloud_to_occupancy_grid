#pragma once

#include "pointcloud_to_occupancy_grid/occupancy_map.hpp"
#include "pointcloud_to_occupancy_grid/types.hpp"

#include <Eigen/Core>

#include <vector>

namespace pointcloud_to_occupancy_grid {

class OfflineMapBuilder {
 public:
  struct Options {
    bool estimate_floor = false;
    double lidar_height = 0.0;
    double floor_height = -1.2;
    double min_threshold_from_floor = 0.5;
    double max_threshold_from_floor = 1.0;
    double usable_scan_range = 50.0;
    double grid_map_resolution = 0.2;
    double occupancy_ratio = 0.3;
    double log_odds_hit = 0.85;
    double log_odds_miss = -0.4;
    double log_odds_max = 3.5;
    double log_odds_min = -2.0;
    std::size_t resize_sample_step = 10;
  };

  OfflineMapBuilder();
  explicit OfflineMapBuilder(Options options);

  bool AddFrame(const FrameData &frame);
  const OccupancyMap &GetMap() const { return map_; }

 private:
  bool ResizeMapForFrame(const FrameData &frame);
  void ConvertFrameTo2DScan(const FrameData &frame);
  void SetWhitePoints(const std::vector<Eigen::Vector2d> &angle_distance_height, const FrameData &frame);
  bool DetectPlaneCoefficients(const FrameData &frame);

  Options options_;
  OccupancyMap map_;
  Eigen::Vector4d floor_coeffs_ = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0);
};

}  // namespace pointcloud_to_occupancy_grid

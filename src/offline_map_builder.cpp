#include "pointcloud_to_occupancy_grid/offline_map_builder.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace pointcloud_to_occupancy_grid {
namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

OfflineMapBuilder::OfflineMapBuilder() : OfflineMapBuilder(Options{}) {}

OfflineMapBuilder::OfflineMapBuilder(Options options)
    : options_(options),
      map_(OccupancyMap::Options{
          static_cast<float>(options.grid_map_resolution),
          static_cast<float>(std::log(options.occupancy_ratio / (1.0 - options.occupancy_ratio))),
          LogOddsParams{static_cast<float>(options.log_odds_hit),
                        static_cast<float>(options.log_odds_miss),
                        static_cast<float>(options.log_odds_max),
                        static_cast<float>(options.log_odds_min)}}) {}

bool OfflineMapBuilder::AddFrame(const FrameData &frame) {
  if (frame.cloud == nullptr || frame.cloud->empty()) {
    return true;
  }

  if (!ResizeMapForFrame(frame)) {
    return false;
  }

  ConvertFrameTo2DScan(frame);
  return true;
}

bool OfflineMapBuilder::ResizeMapForFrame(const FrameData &frame) {
  const Eigen::Isometry3d pose = frame.pose.AsIsometry();
  float min_x = static_cast<float>(frame.pose.translation.x());
  float min_y = static_cast<float>(frame.pose.translation.y());
  float max_x = min_x;
  float max_y = min_y;

  const std::size_t sample_step = std::max<std::size_t>(1, options_.resize_sample_step);
  for (std::size_t i = 0; i < frame.cloud->points.size(); i += sample_step) {
    const auto &point = frame.cloud->points[i];
    const Eigen::Vector3d local_point(point.x, point.y, point.z);
    const double range = local_point.head<2>().norm();
    if (!std::isfinite(range) || range <= 0.01 || range > options_.usable_scan_range) {
      continue;
    }

    const Eigen::Vector3d world_point = pose * local_point;
    min_x = std::min(min_x, static_cast<float>(world_point.x() - 1.0));
    min_y = std::min(min_y, static_cast<float>(world_point.y() - 1.0));
    max_x = std::max(max_x, static_cast<float>(world_point.x() + 1.0));
    max_y = std::max(max_y, static_cast<float>(world_point.y() + 1.0));
  }

  return map_.EnsureBounds(min_x, min_y, max_x, max_y);
}

void OfflineMapBuilder::ConvertFrameTo2DScan(const FrameData &frame) {
  if (options_.estimate_floor) {
    if (!DetectPlaneCoefficients(frame)) {
      floor_coeffs_ = Eigen::Vector4d(0.0, 0.0, 1.0, -options_.floor_height);
    }
  } else {
    floor_coeffs_ = Eigen::Vector4d(0.0, 0.0, 1.0, -options_.floor_height);
  }

  std::vector<std::vector<std::pair<double, double>>> rays(360);
  std::vector<Eigen::Vector2d> angle_distance_height(360, Eigen::Vector2d::Zero());
  const Eigen::Isometry3d pose = frame.pose.AsIsometry();

  for (const auto &point : frame.cloud->points) {
    const Eigen::Vector3d local_point(point.x, point.y, point.z);
    const Eigen::Vector4d homogeneous_point(point.x, point.y, point.z, 1.0);
    const Eigen::Vector2d planar_point(point.x, point.y);
    const double range = planar_point.norm();
    if (!std::isfinite(range) || range > options_.usable_scan_range) {
      continue;
    }

    const double distance_to_floor = homogeneous_point.dot(floor_coeffs_);
    const double angle_deg = std::atan2(planar_point.y(), planar_point.x()) * kRadToDeg;
    const int angle_index = (static_cast<int>(std::round(angle_deg)) + 360) % 360;

    if (distance_to_floor > options_.min_threshold_from_floor) {
      if (distance_to_floor < options_.max_threshold_from_floor) {
        rays[angle_index].emplace_back(range, distance_to_floor);

        const Eigen::Vector3d world_point = pose * local_point;
        map_.SetHitPoint(static_cast<float>(world_point.x()), static_cast<float>(world_point.y()), true,
                         static_cast<float>(distance_to_floor));
      }
    } else if (distance_to_floor > -options_.min_threshold_from_floor) {
      rays[angle_index].emplace_back(range, distance_to_floor);
    }
  }

  // Sort each ray by range ascending so rbegin/rend iterates far-to-near.
  for (auto &ray : rays) {
    std::sort(ray.begin(), ray.end());
  }

  // floor z-height at sensor origin: solve ax+by+cz+d=0 at x=0,y=0 → z = -d/c
  const double floor_z_at_origin = (std::abs(floor_coeffs_[2]) > 1e-6)
      ? -floor_coeffs_[3] / floor_coeffs_[2]
      : -options_.floor_height;
  const double floor_relative_height = floor_z_at_origin;

  for (int angle = 0; angle < 360; ++angle) {
    if (rays[angle].size() < 2) {
      angle_distance_height[angle] =
          Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), floor_relative_height);
      continue;
    }

    for (auto it = rays[angle].rbegin(); it != rays[angle].rend(); ++it) {
      if (it->second < options_.min_threshold_from_floor) {
        angle_distance_height[angle] = Eigen::Vector2d(it->first, it->second);
        break;  // farthest match wins when iterating far-to-near
      }

      auto next_it = it;
      ++next_it;
      if (next_it != rays[angle].rend()) {
        if (it->second > options_.min_threshold_from_floor && next_it->second < options_.min_threshold_from_floor) {
          angle_distance_height[angle] = Eigen::Vector2d(it->first, it->second);
          break;
        }
      } else {
        angle_distance_height[angle] = Eigen::Vector2d(it->first, it->second);
      }
    }
  }

  SetWhitePoints(angle_distance_height, frame);
}

void OfflineMapBuilder::SetWhitePoints(const std::vector<Eigen::Vector2d> &angle_distance_height,
                                       const FrameData &frame) {
  const Eigen::Isometry3d pose = frame.pose.AsIsometry();
  const Eigen::Vector3d origin = frame.pose.translation;

  for (int angle_index = 0; angle_index < 360; ++angle_index) {
    const double angle = static_cast<double>(angle_index) * kDegToRad;
    const double range = angle_distance_height[angle_index].x();
    const double height = angle_distance_height[angle_index].y();
    if (!std::isfinite(range)) {
      continue;
    }

    const Eigen::Vector3d local_point(range * std::cos(angle), range * std::sin(angle), height);
    const Eigen::Vector3d world_point = pose * local_point;

    if (range <= 0.0 || range > options_.usable_scan_range) {
      if (range < 0.1) {
        map_.SetMissPoint(static_cast<float>(world_point.x()), static_cast<float>(world_point.y()),
                          static_cast<float>(origin.x()), static_cast<float>(origin.y()),
                          static_cast<float>(height), static_cast<float>(options_.lidar_height));
      }
      continue;
    }

    map_.SetMissPoint(static_cast<float>(world_point.x()), static_cast<float>(world_point.y()),
                      static_cast<float>(origin.x()), static_cast<float>(origin.y()), static_cast<float>(height),
                      static_cast<float>(options_.lidar_height));
  }
}

bool OfflineMapBuilder::DetectPlaneCoefficients(const FrameData &frame) {
  PointCloudPtr floor_candidates(new PointCloud);
  floor_candidates->reserve(frame.cloud->size());
  for (const auto &point : frame.cloud->points) {
    if (point.z < options_.lidar_height + options_.floor_height) {
      floor_candidates->points.push_back(point);
    }
  }

  floor_candidates->width = static_cast<std::uint32_t>(floor_candidates->points.size());
  floor_candidates->height = 1;
  floor_candidates->is_dense = false;
  if (floor_candidates->size() < 200) {
    return false;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.25);
  segmentation.setInputCloud(floor_candidates);
  segmentation.segment(*inliers, *coefficients);

  if (coefficients->values.size() < 4 || coefficients->values[2] < 0.99f || inliers->indices.size() < 100) {
    return false;
  }

  floor_coeffs_ = Eigen::Vector4d(coefficients->values[0], coefficients->values[1], coefficients->values[2],
                                  coefficients->values[3]);
  return true;
}

}  // namespace pointcloud_to_occupancy_grid

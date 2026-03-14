#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pointcloud_to_occupancy_grid {

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;

struct PoseData {
  double timestamp = 0.0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

  Eigen::Isometry3d AsIsometry() const {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = rotation.normalized().toRotationMatrix();
    transform.translation() = translation;
    return transform;
  }
};

struct FrameSpec {
  std::size_t index = 0;
  PoseData pose;
  std::string pcd_path;
};

struct FrameData {
  std::size_t index = 0;
  PoseData pose;
  PointCloudPtr cloud = PointCloudPtr(new PointCloud);
};

struct RasterMapData {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> data;
};

}  // namespace pointcloud_to_occupancy_grid

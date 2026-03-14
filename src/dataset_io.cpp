#include "pointcloud_to_occupancy_grid/dataset_io.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

namespace pointcloud_to_occupancy_grid {
namespace {

bool IsUnsignedInteger(const std::string &value) {
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char ch) { return std::isdigit(ch) != 0; });
}

bool ParsePoseLine(const std::string &line, PoseData &pose, std::string &error_message) {
  std::stringstream stream(line);
  std::vector<double> values;
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }

  if (values.size() < 8) {
    error_message = "pose line must contain at least 8 numeric columns";
    return false;
  }

  pose.timestamp = values[0];
  pose.translation = Eigen::Vector3d(values[1], values[2], values[3]);
  pose.rotation = Eigen::Quaterniond(values[7], values[4], values[5], values[6]);
  if (pose.rotation.norm() == 0.0) {
    error_message = "pose quaternion is zero";
    return false;
  }

  pose.rotation.normalize();
  return true;
}

}  // namespace

bool LoadFrameSpecs(const std::string &keyframe_pcd_dir, const std::string &pose_file, std::vector<FrameSpec> &frames,
                    std::string &error_message) {
  frames.clear();

  const std::filesystem::path pcd_dir(keyframe_pcd_dir);
  const std::filesystem::path pose_path(pose_file);
  if (!std::filesystem::exists(pcd_dir) || !std::filesystem::is_directory(pcd_dir)) {
    error_message = "keyframe_pcd_dir does not exist or is not a directory: " + keyframe_pcd_dir;
    return false;
  }
  if (!std::filesystem::exists(pose_path)) {
    error_message = "pose_file does not exist: " + pose_file;
    return false;
  }

  std::vector<std::pair<std::size_t, std::filesystem::path>> indexed_pcds;
  for (const auto &entry : std::filesystem::directory_iterator(pcd_dir)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".pcd") {
      continue;
    }

    const std::string stem = entry.path().stem().string();
    if (!IsUnsignedInteger(stem)) {
      error_message = "pcd file name must be an unsigned integer stem: " + entry.path().string();
      return false;
    }

    indexed_pcds.emplace_back(std::stoul(stem), entry.path());
  }

  if (indexed_pcds.empty()) {
    error_message = "no .pcd files found in " + keyframe_pcd_dir;
    return false;
  }

  std::sort(indexed_pcds.begin(), indexed_pcds.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });

  std::ifstream pose_stream(pose_path);
  if (!pose_stream.is_open()) {
    error_message = "failed to open pose file: " + pose_file;
    return false;
  }

  std::vector<PoseData> poses;
  std::string line;
  std::size_t line_index = 0;
  while (std::getline(pose_stream, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) {
      continue;
    }

    PoseData pose;
    if (!ParsePoseLine(line, pose, error_message)) {
      error_message = "invalid pose at line " + std::to_string(line_index + 1) + ": " + error_message;
      return false;
    }

    poses.push_back(pose);
    ++line_index;
  }

  if (poses.size() != indexed_pcds.size()) {
    error_message = "pose count (" + std::to_string(poses.size()) + ") does not match pcd count (" +
                    std::to_string(indexed_pcds.size()) + ")";
    return false;
  }

  frames.reserve(indexed_pcds.size());
  for (std::size_t i = 0; i < indexed_pcds.size(); ++i) {
    FrameSpec frame;
    frame.index = indexed_pcds[i].first;
    frame.pose = poses[i];
    frame.pcd_path = indexed_pcds[i].second.string();
    frames.push_back(std::move(frame));
  }

  return true;
}

bool LoadPointCloud(const std::string &pcd_path, PointCloudPtr &cloud, std::string &error_message) {
  cloud.reset(new PointCloud);
  const int result = pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud);
  if (result != 0) {
    error_message = "failed to load pcd: " + pcd_path;
    return false;
  }

  cloud->is_dense = false;
  if (cloud->height == 0) {
    cloud->height = 1;
  }
  if (cloud->width == 0) {
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  }
  return true;
}

}  // namespace pointcloud_to_occupancy_grid

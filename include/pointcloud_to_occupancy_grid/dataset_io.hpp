#pragma once

#include "pointcloud_to_occupancy_grid/types.hpp"

#include <string>
#include <vector>

namespace pointcloud_to_occupancy_grid {

bool LoadFrameSpecs(const std::string &keyframe_pcd_dir, const std::string &pose_file, std::vector<FrameSpec> &frames,
                    std::string &error_message);

bool LoadPointCloud(const std::string &pcd_path, PointCloudPtr &cloud, std::string &error_message);

}  // namespace pointcloud_to_occupancy_grid

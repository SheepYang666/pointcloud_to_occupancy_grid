#include "pointcloud_to_occupancy_grid/dataset_io.hpp"
#include "pointcloud_to_occupancy_grid/map_export.hpp"
#include "pointcloud_to_occupancy_grid/offline_map_builder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <string>
#include <vector>

namespace pointcloud_to_occupancy_grid {
namespace {

struct AppConfig {
  std::string keyframe_pcd_dir;
  std::string pose_file;
  std::string output_directory;
  std::string pgm_filename;
  std::string yaml_filename;
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
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
};

AppConfig LoadConfig(const rclcpp::Node::SharedPtr &node) {
  AppConfig config;
  config.keyframe_pcd_dir = node->declare_parameter<std::string>(
      "input.keyframe_pcd_dir", "/home/zry/Code/vina_slam_ws/data/map/floor3/keyFrames");
  config.pose_file = node->declare_parameter<std::string>(
      "input.pose_file", "/home/zry/Code/vina_slam_ws/data/map/floor3/keyFrames/alidarState.txt");
  config.output_directory = node->declare_parameter<std::string>(
      "output.directory", "/home/zry/Code/vina_slam_ws/data/map/floor3/occupancy");
  config.pgm_filename = node->declare_parameter<std::string>("output.pgm_filename", "map.pgm");
  config.yaml_filename = node->declare_parameter<std::string>("output.yaml_filename", "map.yaml");
  config.estimate_floor = node->declare_parameter<bool>("projection.esti_floor", false);
  config.lidar_height = node->declare_parameter<double>("projection.lidar_height", 0.0);
  config.floor_height = node->declare_parameter<double>("projection.floor_height", -1.2);
  config.min_threshold_from_floor = node->declare_parameter<double>("projection.min_th_floor", 0.5);
  config.max_threshold_from_floor = node->declare_parameter<double>("projection.max_th_floor", 1.0);
  config.usable_scan_range = node->declare_parameter<double>("projection.usable_scan_range", 50.0);
  config.grid_map_resolution = node->declare_parameter<double>("projection.grid_map_resolution", 0.2);
  config.occupancy_ratio = node->declare_parameter<double>("projection.occupancy_ratio", 0.3);
  config.log_odds_hit = node->declare_parameter<double>("projection.log_odds_hit", 0.85);
  config.log_odds_miss = node->declare_parameter<double>("projection.log_odds_miss", -0.4);
  config.log_odds_max = node->declare_parameter<double>("projection.log_odds_max", 3.5);
  config.log_odds_min = node->declare_parameter<double>("projection.log_odds_min", -2.0);
  config.negate = node->declare_parameter<int>("map_yaml.negate", 0);
  config.occupied_thresh = node->declare_parameter<double>("map_yaml.occupied_thresh", 0.65);
  config.free_thresh = node->declare_parameter<double>("map_yaml.free_thresh", 0.25);
  return config;
}

bool ValidateConfig(const AppConfig &config, std::string &error_message) {
  if (config.keyframe_pcd_dir.empty()) {
    error_message = "input.keyframe_pcd_dir must not be empty";
    return false;
  }
  if (config.pose_file.empty()) {
    error_message = "input.pose_file must not be empty";
    return false;
  }
  if (config.output_directory.empty()) {
    error_message = "output.directory must not be empty";
    return false;
  }
  if (config.pgm_filename.empty() || config.yaml_filename.empty()) {
    error_message = "output file names must not be empty";
    return false;
  }
  if (config.grid_map_resolution <= 0.0) {
    error_message = "projection.grid_map_resolution must be positive";
    return false;
  }
  if (config.occupancy_ratio <= 0.0 || config.occupancy_ratio >= 1.0) {
    error_message = "projection.occupancy_ratio must be between 0 and 1";
    return false;
  }
  if (config.log_odds_hit <= 0.0) {
    error_message = "projection.log_odds_hit must be positive";
    return false;
  }
  if (config.log_odds_miss >= 0.0) {
    error_message = "projection.log_odds_miss must be negative";
    return false;
  }
  if (config.log_odds_max <= 0.0) {
    error_message = "projection.log_odds_max must be positive";
    return false;
  }
  if (config.log_odds_min >= 0.0) {
    error_message = "projection.log_odds_min must be negative";
    return false;
  }
  if (config.log_odds_min >= config.log_odds_max) {
    error_message = "projection.log_odds_min must be less than projection.log_odds_max";
    return false;
  }
  if (config.occupied_thresh <= 0.0 || config.occupied_thresh >= 1.0 || config.free_thresh <= 0.0 ||
      config.free_thresh >= 1.0 || config.free_thresh >= config.occupied_thresh) {
    error_message = "map_yaml thresholds must satisfy 0 < free_thresh < occupied_thresh < 1";
    return false;
  }
  if (config.max_threshold_from_floor < config.min_threshold_from_floor) {
    error_message = "projection.max_th_floor must be >= projection.min_th_floor";
    return false;
  }
  if (config.usable_scan_range <= 0.0) {
    error_message = "projection.usable_scan_range must be positive";
    return false;
  }
  return true;
}

}  // namespace
}  // namespace pointcloud_to_occupancy_grid

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("generate_pgm_from_keyframes");
  const auto logger = node->get_logger();

  const auto config = pointcloud_to_occupancy_grid::LoadConfig(node);
  std::string error_message;
  if (!pointcloud_to_occupancy_grid::ValidateConfig(config, error_message)) {
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  std::vector<pointcloud_to_occupancy_grid::FrameSpec> frame_specs;
  if (!pointcloud_to_occupancy_grid::LoadFrameSpecs(config.keyframe_pcd_dir, config.pose_file, frame_specs,
                                                    error_message)) {
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(logger, "Loaded %zu frame specs from %s", frame_specs.size(), config.keyframe_pcd_dir.c_str());

  pointcloud_to_occupancy_grid::OfflineMapBuilder builder(
      pointcloud_to_occupancy_grid::OfflineMapBuilder::Options{
          config.estimate_floor,           config.lidar_height,
          config.floor_height,             config.min_threshold_from_floor,
          config.max_threshold_from_floor,  config.usable_scan_range,
          config.grid_map_resolution,      config.occupancy_ratio,
          config.log_odds_hit,             config.log_odds_miss,
          config.log_odds_max,             config.log_odds_min,
          10});

  for (std::size_t i = 0; i < frame_specs.size(); ++i) {
    pointcloud_to_occupancy_grid::PointCloudPtr cloud;
    if (!pointcloud_to_occupancy_grid::LoadPointCloud(frame_specs[i].pcd_path, cloud, error_message)) {
      RCLCPP_ERROR(logger, "%s", error_message.c_str());
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }

    pointcloud_to_occupancy_grid::FrameData frame;
    frame.index = frame_specs[i].index;
    frame.pose = frame_specs[i].pose;
    frame.cloud = cloud;
    if (!builder.AddFrame(frame)) {
      RCLCPP_ERROR(logger, "Failed to add frame %zu (%s)", frame.index, frame_specs[i].pcd_path.c_str());
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }

    if ((i + 1) % 10 == 0 || (i + 1) == frame_specs.size()) {
      RCLCPP_INFO(logger, "Processed %zu/%zu frames", i + 1, frame_specs.size());
    }
  }

  const auto raster_map = builder.GetMap().ToRasterMap();
  pointcloud_to_occupancy_grid::MapExportOptions export_options;
  export_options.output_directory = config.output_directory;
  export_options.pgm_filename = config.pgm_filename;
  export_options.yaml_filename = config.yaml_filename;
  export_options.negate = config.negate;
  export_options.occupied_thresh = config.occupied_thresh;
  export_options.free_thresh = config.free_thresh;

  if (!pointcloud_to_occupancy_grid::ExportMapFiles(raster_map, export_options, error_message)) {
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(logger, "Saved occupancy map to %s", config.output_directory.c_str());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

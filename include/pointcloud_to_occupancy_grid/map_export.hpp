#pragma once

#include "pointcloud_to_occupancy_grid/types.hpp"

#include <string>

namespace pointcloud_to_occupancy_grid {

struct MapExportOptions {
  std::string output_directory;
  std::string pgm_filename = "map.pgm";
  std::string yaml_filename = "map.yaml";
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
};

bool ExportMapFiles(const RasterMapData &map, const MapExportOptions &options, std::string &error_message);

}  // namespace pointcloud_to_occupancy_grid

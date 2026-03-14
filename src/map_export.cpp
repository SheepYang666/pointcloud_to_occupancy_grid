#include "pointcloud_to_occupancy_grid/map_export.hpp"

#include <filesystem>
#include <fstream>
#include <vector>

namespace pointcloud_to_occupancy_grid {
namespace {

std::vector<unsigned char> BuildPgmPixels(const RasterMapData &map) {
  std::vector<unsigned char> pixels(static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height), 128);
  for (std::uint32_t y = 0; y < map.height; ++y) {
    for (std::uint32_t x = 0; x < map.width; ++x) {
      const std::size_t source_index = static_cast<std::size_t>(y) * map.width + x;
      const std::size_t target_index = static_cast<std::size_t>(map.height - 1 - y) * map.width + x;
      const int8_t value = map.data[source_index];
      if (value == 0) {
        pixels[target_index] = 255;
      } else if (value == 100) {
        pixels[target_index] = 0;
      } else {
        pixels[target_index] = 128;
      }
    }
  }
  return pixels;
}

}  // namespace

bool ExportMapFiles(const RasterMapData &map, const MapExportOptions &options, std::string &error_message) {
  if (map.width == 0 || map.height == 0 || map.data.empty()) {
    error_message = "map is empty";
    return false;
  }

  const std::filesystem::path output_dir(options.output_directory);
  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    error_message = "failed to create output directory: " + output_dir.string();
    return false;
  }

  const std::filesystem::path pgm_path = output_dir / options.pgm_filename;
  const std::filesystem::path yaml_path = output_dir / options.yaml_filename;

  const std::vector<unsigned char> pixels = BuildPgmPixels(map);
  std::ofstream pgm_stream(pgm_path, std::ios::binary);
  if (!pgm_stream.is_open()) {
    error_message = "failed to open output pgm: " + pgm_path.string();
    return false;
  }

  pgm_stream << "P5\n" << map.width << " " << map.height << "\n255\n";
  pgm_stream.write(reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  pgm_stream.close();
  if (!pgm_stream) {
    error_message = "failed to write pgm file: " + pgm_path.string();
    return false;
  }

  std::ofstream yaml_stream(yaml_path);
  if (!yaml_stream.is_open()) {
    error_message = "failed to open output yaml: " + yaml_path.string();
    return false;
  }

  yaml_stream << "image: " << options.pgm_filename << "\n";
  yaml_stream << "mode: trinary\n";
  yaml_stream << "width: " << map.width << "\n";
  yaml_stream << "height: " << map.height << "\n";
  yaml_stream << "resolution: " << map.resolution << "\n";
  yaml_stream << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n";
  yaml_stream << "negate: " << options.negate << "\n";
  yaml_stream << "occupied_thresh: " << options.occupied_thresh << "\n";
  yaml_stream << "free_thresh: " << options.free_thresh << "\n";
  yaml_stream.close();
  if (!yaml_stream) {
    error_message = "failed to write yaml file: " + yaml_path.string();
    return false;
  }

  return true;
}

}  // namespace pointcloud_to_occupancy_grid

#pragma once

namespace pointcloud_to_occupancy_grid {

struct GridData {
  unsigned int hit_count = 0;
  unsigned int visit_count = 0;
  float height = 10000.0f;
};

}  // namespace pointcloud_to_occupancy_grid

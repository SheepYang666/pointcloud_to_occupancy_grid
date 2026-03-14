#pragma once

namespace pointcloud_to_occupancy_grid {

struct LogOddsParams {
  float log_odds_hit = 0.85f;
  float log_odds_miss = -0.4f;
  float log_odds_max = 3.5f;
  float log_odds_min = -2.0f;
};

struct GridData {
  float log_odds = 0.0f;
  float height = 10000.0f;
  unsigned int visit_count = 0;
};

}  // namespace pointcloud_to_occupancy_grid

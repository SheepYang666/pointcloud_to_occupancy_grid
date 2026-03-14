#include "pointcloud_to_occupancy_grid/subgrid.hpp"

#include <algorithm>
#include <cstring>

namespace pointcloud_to_occupancy_grid {

SubGrid::SubGrid(const SubGrid &other) {
  if (other.grid_data_ == nullptr) {
    return;
  }

  grid_data_ = new GridData[kCellCount];
  std::memcpy(grid_data_, other.grid_data_, sizeof(GridData) * kCellCount);
}

SubGrid &SubGrid::operator=(const SubGrid &other) {
  if (this == &other) {
    return *this;
  }

  GridData *new_data = nullptr;
  if (other.grid_data_ != nullptr) {
    new_data = new GridData[kCellCount];
    std::memcpy(new_data, other.grid_data_, sizeof(GridData) * kCellCount);
  }

  delete[] grid_data_;
  grid_data_ = new_data;
  return *this;
}

SubGrid::SubGrid(SubGrid &&other) noexcept : grid_data_(other.grid_data_) {
  other.grid_data_ = nullptr;
}

SubGrid &SubGrid::operator=(SubGrid &&other) noexcept {
  if (this != &other) {
    delete[] grid_data_;
    grid_data_ = other.grid_data_;
    other.grid_data_ = nullptr;
  }
  return *this;
}

SubGrid::~SubGrid() {
  delete[] grid_data_;
  grid_data_ = nullptr;
}

bool SubGrid::IsEmpty() const {
  return grid_data_ == nullptr;
}

void SubGrid::SetGridHitPoint(bool hit, int sub_x, int sub_y, float height, const LogOddsParams &params) {
  AllocateIfNeeded();

  const int index = sub_x + sub_y * kWidth;
  GridData &cell = grid_data_[index];

  cell.log_odds = std::clamp(cell.log_odds + (hit ? params.log_odds_hit : params.log_odds_miss),
                             params.log_odds_min, params.log_odds_max);
  cell.visit_count += 1;
  if (height < cell.height) {
    cell.height = height;
  }
}

float SubGrid::GetLogOdds(int sub_x, int sub_y) const {
  if (grid_data_ == nullptr) {
    return 0.0f;
  }
  return grid_data_[sub_x + sub_y * kWidth].log_odds;
}

unsigned int SubGrid::GetVisitCount(int sub_x, int sub_y) const {
  if (grid_data_ == nullptr) {
    return 0;
  }
  return grid_data_[sub_x + sub_y * kWidth].visit_count;
}

void SubGrid::AllocateIfNeeded() {
  if (grid_data_ != nullptr) {
    return;
  }

  grid_data_ = new GridData[kCellCount];
}

}  // namespace pointcloud_to_occupancy_grid

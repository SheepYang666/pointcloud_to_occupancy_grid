#include "pointcloud_to_occupancy_grid/subgrid.hpp"

#include <cstring>

namespace pointcloud_to_occupancy_grid {

SubGrid::SubGrid(const SubGrid &other) {
  std::lock_guard<std::mutex> lock(other.data_mutex_);
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
  {
    std::lock_guard<std::mutex> other_lock(other.data_mutex_);
    if (other.grid_data_ != nullptr) {
      new_data = new GridData[kCellCount];
      std::memcpy(new_data, other.grid_data_, sizeof(GridData) * kCellCount);
    }
  }

  std::lock_guard<std::mutex> self_lock(data_mutex_);
  delete[] grid_data_;
  grid_data_ = new_data;
  return *this;
}

SubGrid::~SubGrid() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  delete[] grid_data_;
  grid_data_ = nullptr;
}

bool SubGrid::IsEmpty() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return grid_data_ == nullptr;
}

void SubGrid::SetGridHitPoint(bool hit, int sub_x, int sub_y, float height) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  AllocateIfNeeded();

  const int index = sub_x + sub_y * kWidth;
  GridData &cell = grid_data_[index];

  if (hit) {
    if (height < cell.height) {
      cell.height = height;
      cell.hit_count += 1;
      cell.visit_count += 1;
    }
    return;
  }

  if (cell.hit_count > 3) {
    if (height < cell.height) {
      cell.visit_count += 1;
    }
    return;
  }

  cell.visit_count += 1;
  cell.height = height;
}

void SubGrid::GetHitAndVisit(int sub_x, int sub_y, unsigned int &hit_count, unsigned int &visit_count) const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (grid_data_ == nullptr) {
    hit_count = 0;
    visit_count = 0;
    return;
  }

  const GridData &cell = grid_data_[sub_x + sub_y * kWidth];
  hit_count = cell.hit_count;
  visit_count = cell.visit_count;
}

void SubGrid::AllocateIfNeeded() {
  if (grid_data_ != nullptr) {
    return;
  }

  grid_data_ = new GridData[kCellCount];
}

}  // namespace pointcloud_to_occupancy_grid

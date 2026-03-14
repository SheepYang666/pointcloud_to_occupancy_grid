#pragma once

#include "pointcloud_to_occupancy_grid/grid_data.hpp"

#include <mutex>

namespace pointcloud_to_occupancy_grid {

class SubGrid {
 public:
  static constexpr int kSubGridBits = 4;
  static constexpr int kWidth = (1 << kSubGridBits);
  static constexpr int kCellCount = kWidth * kWidth;

  SubGrid() = default;
  SubGrid(const SubGrid &other);
  SubGrid &operator=(const SubGrid &other);
  ~SubGrid();

  bool IsEmpty() const;
  void SetGridHitPoint(bool hit, int sub_x, int sub_y, float height);
  void GetHitAndVisit(int sub_x, int sub_y, unsigned int &hit_count, unsigned int &visit_count) const;

 private:
  void AllocateIfNeeded();

  mutable std::mutex data_mutex_;
  GridData *grid_data_ = nullptr;
};

}  // namespace pointcloud_to_occupancy_grid

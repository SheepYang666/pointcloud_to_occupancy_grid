#pragma once

#include "pointcloud_to_occupancy_grid/grid_data.hpp"

namespace pointcloud_to_occupancy_grid {

// Not thread-safe. Acceptable for offline single-threaded use.
class SubGrid {
 public:
  static constexpr int kSubGridBits = 4;
  static constexpr int kWidth = (1 << kSubGridBits);
  static constexpr int kCellCount = kWidth * kWidth;

  SubGrid() = default;
  SubGrid(const SubGrid &other);
  SubGrid &operator=(const SubGrid &other);
  SubGrid(SubGrid &&other) noexcept;
  SubGrid &operator=(SubGrid &&other) noexcept;
  ~SubGrid();

  bool IsEmpty() const;
  void SetGridHitPoint(bool hit, int sub_x, int sub_y, float height, const LogOddsParams &params);
  float GetLogOdds(int sub_x, int sub_y) const;
  unsigned int GetVisitCount(int sub_x, int sub_y) const;

 private:
  void AllocateIfNeeded();

  GridData *grid_data_ = nullptr;
};

}  // namespace pointcloud_to_occupancy_grid

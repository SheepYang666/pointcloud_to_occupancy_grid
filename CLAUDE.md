# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Offline ROS2 (C++17) tool that converts VINA-SLAM keyframe point clouds + optimized poses into 2D occupancy maps (`map.pgm` + `map.yaml`) compatible with `nav2_map_server`. It is a one-shot converter, not a long-running mapping node.

Extracted from the `g2p5` module of [lightning-lm](../../../lightning-lm/) (`src/lightning-lm-master/src/core/g2p5/`). The migration removed async threading, ROS `nav_msgs::OccupancyGrid` output, OpenCV export, and `RemoveCarNoise()`, replacing them with synchronous offline batch processing and PGM/YAML file export.

## Build & Run

This package lives inside a colcon workspace. Build from the workspace root:

```bash
# Build (from workspace root, e.g. ~/Code/vina_slam_ws)
colcon build --packages-select pointcloud_to_occupancy_grid

# Source the workspace
source install/setup.bash

# Run with default config
ros2 launch pointcloud_to_occupancy_grid start.launch.py

# Run with custom params
ros2 launch pointcloud_to_occupancy_grid start.launch.py params_file:=/path/to/custom.yaml

# Run directly (without launch file)
ros2 run pointcloud_to_occupancy_grid generate_pgm_from_keyframes --ros-args --params-file config/default.yaml
```

No unit tests exist yet. `BUILD_TESTING` is wired in CMakeLists.txt with `ament_lint_auto` but no test targets are defined.

## Architecture

Two build targets:
- **`pointcloud_to_occupancy_grid_core`** — static library, ROS-agnostic algorithm code (Eigen + PCL only)
- **`generate_pgm_from_keyframes`** — executable entry point, links core lib + rclcpp

Data flow: load PCD files + pose file → per-frame 3D→2D projection with height filtering → sparse occupancy grid accumulation → raster export.

### Key source files by responsibility

| Layer | Header | Source | Role |
|-------|--------|--------|------|
| Types | `types.hpp` | — | `PoseData`, `FrameSpec`, `FrameData`, `RasterMapData` |
| I/O | `dataset_io.hpp` | `dataset_io.cpp` | Load PCD directory + pose text file, match by index order |
| Grid cell | `grid_data.hpp` | — | Per-cell hit/visit counts and height |
| Subgrid | `subgrid.hpp` | `subgrid.cpp` | 16×16 lazily-allocated cell block (not thread-safe; mutex removed for offline use) |
| Map | `occupancy_map.hpp` | `occupancy_map.cpp` | Sparse 2D grid, dynamic resizing, hit/miss marking, rasterization |
| Algorithm | `offline_map_builder.hpp` | `offline_map_builder.cpp` | Core pipeline: floor estimation, obstacle filtering, ray-cast free space |
| Export | `map_export.hpp` | `map_export.cpp` | Write PGM (P5 binary) + YAML |
| Entry | — | `generate_pgm_from_keyframes.cpp` | ROS2 node: param loading, validation, orchestration |

### Algorithm essentials

- Optional per-frame RANSAC floor plane estimation (via PCL `SACSegmentation`)
- Points filtered by height band above floor (`min_th_floor` .. `max_th_floor`) and planar range (`usable_scan_range`)
- Free-space filled by angle-bin ray casting from sensor origin to nearest obstacle per bin
- Trinary output: occupied=100, free=0, unknown=-1 (maps to PGM black/white/gray)

## Input Contract

- **PCD files**: must have unsigned-integer stems (e.g. `0.pcd`, `1.pcd`, …), `PointXYZI` type, sorted numerically
- **Pose file**: whitespace-delimited, 8+ columns per line: `timestamp tx ty tz qx qy qz qw` (VINA-SLAM `alidarState.txt` format)
- Matching is by index order (Nth PCD ↔ Nth pose line), not by timestamp

## Dependencies

`rclcpp`, `Eigen3`, `PCL` (common, io, segmentation), `ament_cmake`, `launch`, `launch_ros`

## Configuration

All parameters live under `generate_pgm_from_keyframes.ros__parameters` in `config/default.yaml`. Key tuning knobs:

- `projection.grid_map_resolution` — cell size in meters (default 0.05)
- `projection.min_th_floor` / `max_th_floor` — obstacle height band above floor
- `projection.usable_scan_range` — max planar range for projection
- `projection.occupancy_ratio` — hit/visit ratio threshold for occupied classification
- `projection.esti_floor` — enable per-frame RANSAC floor estimation (false = use fixed `floor_height`)

Note: C++ fallback defaults in `generate_pgm_from_keyframes.cpp` differ from `config/default.yaml` for several parameters (e.g. `grid_map_resolution` 0.2 vs 0.05, `floor_height` -1.2 vs -0.15, `min_th_floor` 0.5 vs 0.05). Always run with `--params-file` or via the launch file.

`resize_sample_step` (hardcoded to 10 in the executable) controls how many points are sampled when pre-sizing the map for a new frame. It is not exposed as a ROS parameter.

## Conventions

- All headers in `include/pointcloud_to_occupancy_grid/`, all sources in `src/`
- Namespace: `pointcloud_to_occupancy_grid`
- `PascalCase` classes, `snake_case` files and variables, no trailing underscore on struct fields (diverges from lightning-lm's `field_` convention)
- Error handling via `bool` return + `std::string& error_message` out-parameter pattern
- Coordinate frame: each PCD is in local LiDAR frame; each pose transforms to global frame; pose translation = LiDAR origin
- Planar range means `sqrt(dx² + dy²)` (not 3D norm) — this is intentional for correct 2D projection
- Floor z-height at sensor origin: `z = -d/c` from plane `ax+by+cz+d=0` (not `d` directly — only coincides for flat floors)
- `SubGrid::SetGridHitPoint` hit branch: `hit_count` and `visit_count` intentionally only increment together when a new minimum height is found — this is NOT a bug. The ratio `hit_count/visit_count` measures height-confirmation consistency, not raw hit frequency. Unconditionally incrementing `visit_count` dilutes the ratio and causes obstacles to disappear.
- Commit style: short imperative subject, optionally scoped (e.g. `projection: fix height band filter`)

# pointcloud_to_occupancy_grid

`pointcloud_to_occupancy_grid` is a ROS2 package that converts saved keyframe point clouds plus optimized poses into a 2D occupancy map in `map.pgm + map.yaml` format.

The package is intentionally offline-only in its current form:

- input: `keyFrames/*.pcd` and `alidarState.txt`
- processing: accumulate all frames into a 2.5D occupancy grid
- output: ROS map_server compatible `pgm/yaml`

This document is written as a handoff note for both engineers and AI tools. It explains what was added in commit `c253d21`, why the package is structured this way, and where to modify it for future changes.

## 1. What Was Added

Before commit `c253d21`, this repository did not contain a working ROS2 package implementation. The commit turned the repository into a standalone `ament_cmake` package with:

- a buildable ROS2 package skeleton
- an offline executable: `generate_pgm_from_keyframes`
- a launch entrypoint: `start.launch.py`
- a default parameter file: `config/default.yaml`
- a lightweight 2.5D occupancy-grid core adapted from the `g2p5` logic in `lightning-lm`
- dataset loading logic for VINA-SLAM style `keyFrames/*.pcd` and `alidarState.txt`
- raster export to `map.pgm` and `map.yaml`

The package does not subscribe to ROS topics and does not maintain a long-running mapping node. It is a one-shot converter.

## 2. Why This Design

The original request started from a different assumption: subscribing to a point-cloud topic and directly generating a `pgm`.

After inspecting the upstream code and the available dataset, the real requirement became:

- read keyframe point clouds from disk
- read keyframe poses from disk
- transform each local frame into a common global frame
- build one offline occupancy map

That requirement changes the architecture substantially. A topic-only node is not sufficient unless the incoming point cloud is already globally aligned and the sensor origin for each frame is known. Since the available data was already stored as:

- `keyFrames/0.pcd ... N.pcd`
- `keyFrames/alidarState.txt`

the cleanest implementation was an offline generator.

## 3. Repository Layout

The main source files added by the implementation are:

- `package.xml`
- `CMakeLists.txt`
- `config/default.yaml`
- `launch/start.launch.py`
- `include/pointcloud_to_occupancy_grid/types.hpp`
- `include/pointcloud_to_occupancy_grid/dataset_io.hpp`
- `include/pointcloud_to_occupancy_grid/grid_data.hpp`
- `include/pointcloud_to_occupancy_grid/subgrid.hpp`
- `include/pointcloud_to_occupancy_grid/occupancy_map.hpp`
- `include/pointcloud_to_occupancy_grid/offline_map_builder.hpp`
- `include/pointcloud_to_occupancy_grid/map_export.hpp`
- `src/dataset_io.cpp`
- `src/subgrid.cpp`
- `src/occupancy_map.cpp`
- `src/offline_map_builder.cpp`
- `src/map_export.cpp`
- `src/generate_pgm_from_keyframes.cpp`

Their responsibilities are:

### Entry and packaging

- `package.xml`
  Declares ROS2 package metadata and runtime dependencies.

- `CMakeLists.txt`
  Builds the core library and the offline executable, and installs `config/` and `launch/`.

- `launch/start.launch.py`
  Loads the node with a parameter file and provides a standard `ros2 launch` entrypoint.

- `config/default.yaml`
  Default runtime configuration for a sample floor-map dataset.

### Input/data layer

- `types.hpp`
  Common data structures such as `PoseData`, `FrameSpec`, `FrameData`, and `RasterMapData`.

- `dataset_io.hpp` / `dataset_io.cpp`
  Reads:
  - sorted `*.pcd` files from a directory
  - pose rows from `alidarState.txt`
  - individual PCD files into `pcl::PointCloud<pcl::PointXYZI>`

### Occupancy-grid core

- `grid_data.hpp`
  Minimal cell-level statistics: hit count, visit count, and height.

- `subgrid.hpp` / `subgrid.cpp`
  Sparse subgrid container with per-cell update logic.

- `occupancy_map.hpp` / `occupancy_map.cpp`
  The main 2D sparse occupancy-map representation. Handles:
  - dynamic map resizing
  - hit/miss updates
  - conversion to final raster occupancy data

- `offline_map_builder.hpp` / `offline_map_builder.cpp`
  The main algorithm layer. Handles:
  - frame-wise map growth
  - floor modeling
  - obstacle selection
  - free-space ray casting

### Output layer

- `map_export.hpp` / `map_export.cpp`
  Exports a `RasterMapData` result into:
  - binary PGM
  - ROS-compatible YAML metadata

### Process entrypoint

- `generate_pgm_from_keyframes.cpp`
  Reads parameters, validates them, loads all frames, runs the builder, and writes output.

## 4. Runtime Data Flow

The current end-to-end flow is:

1. Launch or run the executable.
2. Load ROS2 parameters from `config/default.yaml` or a custom file.
3. Enumerate `keyframe_pcd_dir/*.pcd`.
4. Sort PCDs by the numeric filename stem.
5. Read `pose_file` line by line.
6. Match pose lines to sorted PCD files by index order.
7. For each frame:
   - load the PCD file
   - parse the frame pose
   - expand the map bounds if necessary
   - project valid obstacle points to occupancy cells
   - cast free-space rays from the frame origin
8. Convert the sparse internal map to a raster occupancy image.
9. Export `map.pgm` and `map.yaml`.
10. Exit.

This is deliberately single-process and single-purpose. There is no incremental publish loop and no long-term process state beyond the lifetime of one run.

## 5. Input Contract

The current implementation assumes the dataset format below.

### Point cloud directory

`input.keyframe_pcd_dir` must point to a directory containing files like:

```text
0.pcd
1.pcd
2.pcd
...
122.pcd
```

Important constraints:

- file extension must be `.pcd`
- file name stem must be an unsigned integer
- files are sorted numerically, not lexicographically
- the point type is expected to be `pcl::PointXYZI`

### Pose file

`input.pose_file` must point to a text file containing one pose per line.

Minimum supported row format:

```text
timestamp tx ty tz qx qy qz qw
```

The implementation also supports VINA-SLAM style longer rows such as:

```text
timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz ...
```

Only the first 8 values are required and used.

### Matching rule

The code does not match by timestamp and does not parse a frame id from the pose file.

It assumes:

- pose line 0 belongs to the numerically smallest PCD file
- pose line 1 belongs to the next PCD file
- ...

If pose count and PCD count do not match, the node exits with an error.

## 6. Coordinate Assumptions

This package assumes the following:

- each `*.pcd` is a local keyframe cloud in the LiDAR frame
- each row in `alidarState.txt` provides that frame pose in the global frame
- the pose translation is the LiDAR origin for that frame

The implementation transforms local obstacle points into the world frame using the frame pose. Free-space rays are also emitted from the frame origin.

This means the current package is appropriate for:

- saved SLAM keyframes
- saved localization submaps with known frame poses
- offline map generation after optimization

It is not yet a general online topic-based occupancy mapper.

## 7. Core Algorithm

The algorithm is intentionally close to the `g2p5` approach from `lightning-lm`, but simplified for offline use.

### 7.1 Dynamic map growth

Before integrating a frame, the map is resized to fit:

- the frame origin
- sampled transformed points from the frame

This avoids pre-allocating a full dense world map.

### 7.2 Floor model

For each frame, the algorithm defines a plane used to measure "distance above ground."

Two modes exist:

- `esti_floor: false`
  Use a fixed horizontal plane derived from `floor_height`.

- `esti_floor: true`
  Run a per-frame RANSAC plane fit on candidate low points and use the fitted plane if it passes sanity checks.

The fixed-plane fallback is:

```text
z = floor_height
```

in the LiDAR frame.

### 7.3 Obstacle filtering

Each point is converted to:

- planar range from the LiDAR origin
- distance to the floor plane
- angle bin from 0 to 359 degrees

A point is treated as an obstacle hit only if:

- planar range is finite and within `usable_scan_range`
- height above floor is greater than `min_th_floor`
- height above floor is less than `max_th_floor`

### 7.4 Free-space filling

For each angle bin, the code builds a distance-height profile. It then chooses a representative endpoint and marks cells along the ray from the sensor origin to that endpoint as free.

This is how the output contains both:

- occupied cells
- known free cells

instead of only black obstacle pixels.

### 7.5 Raster export

The sparse internal map is converted to trinary occupancy:

- occupied -> 100
- free -> 0
- unknown -> -1

Then exported to PGM values:

- occupied -> black (`0`)
- free -> white (`255`)
- unknown -> gray (`128`)

## 8. Floor Estimation: Current Behavior

If `projection.esti_floor` is set to `true`, the code executes a per-frame plane fit in `OfflineMapBuilder::DetectPlaneCoefficients`.

The current implementation:

1. collects candidate points satisfying:
   `point.z < lidar_height + floor_height`
2. runs:
   - `pcl::SACSegmentation`
   - `SACMODEL_PLANE`
   - `SAC_RANSAC`
3. accepts the plane only if:
   - the normal's `z` component is at least `0.99`
   - inlier count is at least `100`

If the estimate fails, the code falls back to the fixed horizontal floor plane.

This works reasonably when:

- the LiDAR is approximately level
- the floor is approximately planar
- each frame contains enough ground points

This is not a robust general terrain estimator. For ramps, stairs, large pitch changes, or partial floor observations, a better floor model would be needed.

## 9. Important Parameters

The main tuning parameters are:

### Input and output

- `input.keyframe_pcd_dir`
  Directory of keyframe PCDs.

- `input.pose_file`
  Pose file, usually `keyFrames/alidarState.txt`.

- `output.directory`
  Output folder for `pgm/yaml`.

- `output.pgm_filename`
- `output.yaml_filename`

### Projection and occupancy

- `projection.esti_floor`
  Enable or disable per-frame plane estimation.

- `projection.lidar_height`
  Height of the LiDAR above ground, in meters.

- `projection.floor_height`
  Default floor height in the LiDAR frame. If the LiDAR is 0.2m above the ground, this should usually be `-0.2`.

- `projection.min_th_floor`
  Minimum obstacle height above the floor. Smaller values keep low obstacles; larger values suppress floor noise.

- `projection.max_th_floor`
  Maximum obstacle height above the floor. Useful for excluding ceilings or very high structure.

- `projection.usable_scan_range`
  Max planar range used for occupancy projection. Smaller values are often better indoors.

- `projection.grid_map_resolution`
  Cell size in meters. `0.05` is appropriate for indoor maps; `0.1` is often enough for larger maps.

- `projection.occupancy_ratio`
  Threshold used when deciding whether a visited cell is occupied.

### YAML export

- `map_yaml.negate`
- `map_yaml.occupied_thresh`
- `map_yaml.free_thresh`

These follow the standard map-server interpretation.

## 10. Recommended Vehicle Geometry Settings

For a robot where:

- LiDAR is `0.2m` above the wheel-ground contact plane
- chassis bottom is on the ground reference
- vehicle top is about `2.0m`

the recommended starting point is:

```yaml
projection:
  esti_floor: false
  lidar_height: 0.2
  floor_height: -0.2
  min_th_floor: 0.05
  max_th_floor: 2.0
  usable_scan_range: 25.0
  grid_map_resolution: 0.05
  occupancy_ratio: 0.3
```

Why:

- `lidar_height = 0.2`
  matches the physical sensor height
- `floor_height = -0.2`
  means the floor is 0.2m below the LiDAR origin
- `min_th_floor = 0.05`
  suppresses ground noise while preserving low obstacles
- `max_th_floor = 2.0`
  keeps the vertical volume that the vehicle itself cannot pass through

## 11. Verification That Was Performed

The implementation was verified with the following commands:

```bash
cd /home/zry/Code/vina_slam_ws
colcon build --packages-select pointcloud_to_occupancy_grid
```

```bash
source /home/zry/Code/vina_slam_ws/install/setup.bash
ros2 run pointcloud_to_occupancy_grid generate_pgm_from_keyframes --ros-args \
  --params-file /home/zry/Code/vina_slam_ws/src/pointcloud_to_occupancy_grid/config/default.yaml
```

```bash
source /home/zry/Code/vina_slam_ws/install/setup.bash
ros2 launch pointcloud_to_occupancy_grid start.launch.py
```

There was also a smoke test using a temporary pose file with only the first 8 columns retained from `alidarState.txt` to verify that long VINA-SLAM rows are not required.

## 12. Porting Corrections Made During Implementation

Three important correctness fixes were applied while adapting the upstream logic:

### 12.1 No fake free-space rays for empty bins

Sparse or empty angle bins should not produce artificial free space. The implementation now skips bins without a valid endpoint.

### 12.2 Planar range only

The 2D ray endpoint must be based on planar distance, not the full 3D norm. Otherwise, points above or below the LiDAR would incorrectly extend free-space rays in `x/y`.

### 12.3 No row wrapping during free-cell expansion

When exporting free cells to a raster, neighbor expansion now stays within the same row and does not accidentally write into adjacent rows.

These fixes matter because they directly affect the topology of the final occupancy map.

## 13. How To Run

### Run directly

```bash
cd /home/zry/Code/vina_slam_ws
source install/setup.bash
ros2 run pointcloud_to_occupancy_grid generate_pgm_from_keyframes --ros-args \
  --params-file /home/zry/Code/vina_slam_ws/src/pointcloud_to_occupancy_grid/config/default.yaml
```

### Run with launch

```bash
cd /home/zry/Code/vina_slam_ws
source install/setup.bash
ros2 launch pointcloud_to_occupancy_grid start.launch.py
```

### Run with a custom parameter file

```bash
ros2 launch pointcloud_to_occupancy_grid start.launch.py \
  params_file:=/absolute/path/to/custom.yaml
```

## 14. How To Modify This Package

This section is the most important if the next reader is an AI agent.

### If you want to support a different pose-file format

Edit:

- `src/dataset_io.cpp`

Primary functions:

- `ParsePoseLine`
- `LoadFrameSpecs`

### If you want to support a different point type

Edit:

- `include/pointcloud_to_occupancy_grid/types.hpp`
- `src/dataset_io.cpp`
- `src/offline_map_builder.cpp`

### If you want to change occupancy behavior

Edit:

- `src/offline_map_builder.cpp`
- `src/occupancy_map.cpp`

Typical changes:

- change obstacle height thresholds
- change angle binning resolution
- change free-space ray logic
- change occupancy decision thresholding

### If you want to make it subscribe to ROS topics

Do not start by modifying `offline_map_builder.cpp`.

Instead:

1. keep the current builder as the algorithm core
2. add a ROS2 adapter node that:
   - subscribes to point clouds
   - receives per-frame pose information
   - converts those into `FrameData`
   - feeds the builder
3. decide whether the online node:
   - accumulates forever
   - publishes intermediate maps
   - saves periodically or via service

This package is already split so that this adapter can be added later without rewriting the core map logic.

### If you want to estimate floor more robustly

Edit:

- `src/offline_map_builder.cpp`

Likely improvements:

- widen or redesign candidate-point selection
- keep the previous valid floor plane instead of re-estimating every frame
- use a local window of frames instead of one frame
- support sloped floors

## 15. Known Limitations

- offline only
- assumes a pose is available for every PCD
- assumes all PCD files use the same point type
- floor estimation is still a simple planar heuristic
- no unit tests yet
- no visualization tool beyond the exported `pgm`

## 16. Commit and Traceability

The main implementation commit is:

```text
c253d21 feat: add offline keyframe occupancy map generator
```

To inspect the full patch and commit message:

```bash
git -C /home/zry/Code/vina_slam_ws/src/pointcloud_to_occupancy_grid show c253d21
```

To inspect only the files touched by that commit:

```bash
git -C /home/zry/Code/vina_slam_ws/src/pointcloud_to_occupancy_grid show --stat c253d21
```

To let another AI read the code and this document together, the minimum useful context is:

- this `README.md`
- `config/default.yaml`
- `src/generate_pgm_from_keyframes.cpp`
- `src/dataset_io.cpp`
- `src/offline_map_builder.cpp`
- `src/occupancy_map.cpp`

That is the smallest high-signal set for understanding or extending the current implementation.

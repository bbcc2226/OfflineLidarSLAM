# Offline LiDAR SLAM

An offline ROS 2 LiDAR-inertial SLAM pipeline for recorded KITTI-style sensor data. It reads LiDAR, IMU, and GPS messages from a ROS 2 bag, estimates motion with NDT/GICP scan matching and an error-state Kalman filter, then builds a GPS-constrained pose graph with loop-closure correction.

> **Status:** experimental/research code. The package provides an installed ROS 2 executable and configurable input topics, but some development tests still contain local paths. There is no launch file yet.

## Pipeline

```text
ROS 2 bag (SQLite3)
  ├── /kitti/velodyne_points ─► filtering ─► NDT + GICP odometry
  ├── /kitti/imu             ─► ESKF prediction ───────────┤
  └── /kitti/gps/fix         ─► WGS84 to local ENU ───────┘
                                                            │
                                                            ▼
                                                        keyframes
                                                            │
                       ┌────────────────────────────────────┴──────────┐
                       ▼                                               ▼
              local/global g2o graph                          loop detection
              (LIO + GPS constraints)                     (search + NDT/GICP)
                       └─────────────────────┬─────────────────────────┘
                                             ▼
                                 corrected map and trajectory
```

Main components:

- **`offline_lidar_slam_node`** (`src/main.cpp`) is the installed command-line entry point. It loads configuration, starts the pipeline, waits for completion, and handles `Ctrl+C` shutdown.
- **`SensorDataPlayer`** reads the supported streams from an SQLite3 ROS 2 bag.
- **`SlamFrontEnd`** buffers measurements, downsamples scans, runs odometry, and creates keyframes.
- **`LidarOdodmetry`** estimates local motion using incremental NDT with optional GICP refinement.
- **`ESKF`** propagates state with IMU measurements and applies LiDAR pose updates.
- **`GeoConverter`** converts GPS coordinates to a local ENU frame.
- **`Backend`** performs sliding-window and global g2o optimization using LIO, GPS, and loop-closure edges.
- **`VoxelizedMap`** builds the point-cloud map and removes sparse voxels.

## Input data

The default configuration expects a ROS 2 **SQLite3** bag directory with these topics:

| Topic | Message type | Purpose |
|---|---|---|
| `/kitti/velodyne_points` | `sensor_msgs/msg/PointCloud2` | LiDAR scans |
| `/kitti/imu` | `sensor_msgs/msg/Imu` | ESKF prediction |
| `/kitti/gps/fix` | `sensor_msgs/msg/NavSatFix` | ENU positions and graph constraints |

LiDAR messages must contain `x`, `y`, and `z` fields as 32-bit floats; an `intensity` field is allowed but not required. Invalid (`NaN`/infinite) points are discarded. All streams must share a clock, have ordered timestamps, and overlap in time. Although GPS and IMU appear optional at the API level, the current synchronization logic requires both to advance before processing a LiDAR frame.

## Dependencies

- CMake 3.8+ and a C++17-capable compiler
- ROS 2: `ament_cmake`, `rclcpp`, `sensor_msgs`, `nav_msgs`, `rosbag2_cpp`, and `rosbag2_storage`
- Eigen3, PCL, oneTBB, yaml-cpp, fmt, and g2o
- `ament_cmake_gtest`/GoogleTest when building tests

The Python utilities additionally use some combination of `rclpy`, `rosbag2_py`, NumPy, Matplotlib, OpenCV, and Open3D.

## Build

Place this package under a ROS 2 workspace's `src` directory, then run from the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/<ros-distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select offline_lidar_slam --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

If CMake cannot find g2o, ensure its installation exports `g2o_core`, `g2o_stuff`, `g2o_types_slam3d`, and `g2o_solver_eigen`.

## Run the pipeline

### 1. Prepare a compatible bag

Inspect an existing bag:

```bash
ros2 bag info /absolute/path/to/bag_directory
```

To convert a KITTI raw drive, arrange the extracted data as expected by `script/kitti_unsynced_ros2bag.py`:

```text
data/<drive>/
├── oxts/
│   ├── data/*.txt
│   └── timestamps.txt
└── velodyne_points/
    ├── data/*.txt
    └── timestamps.txt
```

Run the converter with the KITTI drive directory and desired output bag path:

```bash
cd ~/ros2_ws/src/offline_lidar_slam
python3 script/kitti_unsynced_ros2bag.py \
  ./data/2011_10_03_drive_0027 \
  /absolute/path/to/2011_10_03_drive_0027_bag
```

Use `--imu-topic`, `--gps-topic`, `--lidar-topic`, or `--storage-id` when creating a bag with non-default names or storage.

### 2. Configure the run

Edit [`include/Config/Config.yaml`](include/Config/Config.yaml), at minimum replacing:

```yaml
DataLoader:
  ros_bag_path: "/absolute/path/to/bag_directory"
  storage_id: "sqlite3"
  lidar_topic: "/kitti/velodyne_points"
  imu_topic: "/kitti/imu"
  gps_topic: "/kitti/gps/fix"

FrontEnd:
  lio_dir_path: "./LIO_results"
```

The executable uses the installed copy of this file by default. After changing the source copy, rebuild the package so Colcon installs the new version. For experiments, passing a separate YAML file with `--config` avoids rebuilding for configuration-only changes.

Relative bag and output paths are resolved from the directory where the executable is launched. Absolute paths are recommended for reproducible runs.

### 3. Start SLAM

Rebuild after changing source files, source the workspace, and run:

```bash
cd ~/ros2_ws
colcon build --packages-select offline_lidar_slam
source install/setup.bash
ros2 run offline_lidar_slam offline_lidar_slam_node
```

To load a configuration outside the installed package share directory:

```bash
ros2 run offline_lidar_slam offline_lidar_slam_node --config /absolute/path/to/Config.yaml
```

Show the command-line options with:

```bash
ros2 run offline_lidar_slam offline_lidar_slam_node --help
```

The process exits after the bag is consumed and the back end drains its keyframe queue. Use `Ctrl+C` for early shutdown.

For visualization, start `rviz2` in another sourced terminal, set the fixed frame to `map`, and add displays for the topics below.

## Outputs

### ROS 2 topics

| Topic | Message type | Description |
|---|---|---|
| `/voxel_map` | `sensor_msgs/msg/PointCloud2` | Current optimized voxel map; transient-local durability |
| `/trajectory` | `nav_msgs/msg/Path` | Optimized SLAM trajectory |
| `/rtk_trajectory` | `nav_msgs/msg/Path` | GPS trajectory after yaw alignment |

All messages use the `map` frame.

### Files

- `LIO_results/cloud_XXXXXX.ply` — filtered local keyframe clouds when `save_lio_frame: true`.
- `LO_results/cloud_XXXXXX.ply` — odometry frames when `save_lo_frame: true`.
- `key_frames.jsonl` — timestamps, PLY paths, RTK data, LIO poses, and optimized poses. It appears in the working directory once frames leave the local optimization window.
- `/tmp/loop_submap_kf<ID>.ply`, `/tmp/loop_curr_raw_kf<ID>.ply`, and `/tmp/loop_curr_aligned_kf<ID>.ply` — loop-closure diagnostics when debug output is enabled.

## Configuration

| Section | Controls |
|---|---|
| `ESKF` | Process and LiDAR measurement noise |
| `FrontEnd` | Sensor-buffer watermarks and keyframe output directory |
| `GeoConverter` | WGS84 constants |
| `LidarOdometry` | Voxel/NDT resolution, GICP, range limits, and pose-jump rejection |
| `DataLoader` | Bag path and ground filtering |
| `General` | Map resolution, output toggles, LIO-only mode, and debug output |
| `LoopClosure` | Search radius, separation, candidates, fitness gates, and interval |
| `Optimizer` | Edge weights, local window, and iteration counts |

Two misspelled keys are part of the current API and must retain their spelling: `remove_groud` and `local_optimization_widnow_size`.

Typical tuning directions:

- Increase voxel sizes to reduce computation and memory at the cost of detail.
- Adjust NDT resolution for the scale and density of the sensor data.
- Disable `scan_matching_use_gicp_fine_alignment` to use NDT alone.
- Tighten pose-jump limits to reject unstable registration, or loosen them for larger inter-scan motion.
- Increase loop-closure frame separation to avoid matching nearby observations.
- `using_LIO_only: true` builds the map from LIO poses; GPS is still required by the current front end.

## Tests

```bash
cd ~/ros2_ws
colcon test --packages-select offline_lidar_slam
colcon test-result --verbose
```

The executable above is the normal way to run SLAM. The test targets are retained for development and regression checks. Several are integration/debugging programs rather than isolated unit tests and contain hard-coded input paths:

- `test_slam` is the legacy end-to-end integration runner; new runs should use `offline_lidar_slam_node`.
- `test_dataloader`, `test_lo`, and `test_fe` expect local KITTI/bag data.
- `test_loop_closure_ndt` replays diagnostics from an earlier loop-closure attempt.
- `test_ndt` contains scan-registration experiments using saved clouds.

## Utility scripts

- `kitti_unsynced_ros2bag.py` — converts KITTI OXTS and Velodyne text exports to a ROS 2 bag.
- `cloud_publisher.py` — publishes keyframe clouds for ROS visualization.
- `submap_publisher.py` and `loop_closure_inspector.py` — inspect loop-closure data.
- `associate_keyframes_images.py` — associates keyframes with KITTI images.
- `project_keyframes_to_image.py` — projects LiDAR keyframes into images.
- `visualize_colored_ply.py` and `visualize_colored_ply_batch.py` — inspect colored PLY files.
- `plot_oxts_enu.py` and `plot_timestamp_gaps.py` — inspect GPS tracks and sensor timing.

Run scripts with `--help` where supported:

```bash
python3 script/cloud_publisher.py --help
python3 script/project_keyframes_to_image.py --help
```

## Repository layout

```text
offline_lidar_slam/
├── include/                # Interfaces, config, publishers, bundled Sophus headers
│   └── Config/Config.yaml  # Runtime configuration
├── src/
│   ├── main.cpp            # Installed offline_lidar_slam_node entry point
│   └── ...                 # Front end, odometry, ESKF, optimization, and mapping
├── script/                 # Conversion, visualization, and debugging tools
├── test/                   # Unit, integration, and replay/debug tests
├── CMakeLists.txt
└── package.xml
```

## Known limitations

- Several development tests still contain hard-coded dataset or `/tmp` paths.
- The package does not yet provide a ROS 2 launch file.
- Synchronization assumes ordered, overlapping streams and does not interpolate measurements.
- The back end waits for GPS/LIO yaw alignment; this currently needs 50 paired samples and sufficient motion.
- Saved PLY keyframes are required when constructing the voxel map.
- `package.xml` is incomplete relative to CMake, so `rosdep` may not install everything.
- Large bags can use substantial CPU, memory, and disk space.

## License

No license has been declared. Add a `LICENSE` file and update `package.xml` before distribution or reuse.

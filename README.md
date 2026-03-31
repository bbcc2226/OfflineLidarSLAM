# Offline LiDAR SLAM

An offline LiDAR-based Simultaneous Localization and Mapping (SLAM) system built with ROS2 and C++. This system processes pre-recorded sensor data from ROS2 bags to perform point cloud registration, trajectory estimation, and 3D map generation.

## Overview

This project implements a complete SLAM pipeline that includes:

- **Data Loading**: ROS2 bag reader supporting LiDAR, IMU, and GPS data streams
- **Frontend Lidar Odometry**: Frame-to-frame point cloud alignment using NDT (Normal Distributions Transform) registration, and keyframe detection, submap generation
- **State Estimation**: ESKF (Error State Kalman Filter) for pose estimation
- **Lidar Odometry**: Frame-to-frame point cloud alignment with keyframe detection
- **Map Management**: Voxelized mapping for efficient memory usage



## Project Structure

```
offline_lidar_slam/
├── include/
│   ├── Config/
│   │   └── Config.yaml          # Configuration parameters (YAML format)
│   ├── Common.hpp               # Common utilities and functions
│   ├── Config.hpp               # Configuration parameters (C++ header)
│   ├── DataLoader.hpp           # ROS2 bag data loading interface
│   ├── DataType.hpp             # Custom data types (KeyFrame, IMUdata, etc.)
│   ├── ESKF.hpp                 # Error State Kalman Filter implementation
│   ├── FrontEnd.hpp             # Frontend SLAM processing
│   ├── GeoConverter.hpp          # Geographic coordinate conversion
│   ├── LidarOdometry.hpp         # LiDAR odometry estimation
│   ├── NDT_INC.hpp               # NDT algorithm implementation
│   ├── SlamProcess.hpp           # Main SLAM process orchestrator
│   ├── TicToc.hpp                # Timing utilities
│   ├── VoxelFilter.hpp           # Point cloud voxel filtering
│   └── sophus/                   # Sophus library for SE3 transformations
├── src/
│   ├── Common.cpp
│   ├── DataLoader.cpp            # Loads sensor data from ROS2 bags
│   ├── ESKF.cpp                  # Kalman filter for state estimation
│   ├── FrontEnd.cpp              # Frontend processing pipeline
│   ├── GeoConverter.cpp
│   ├── LidarOdometry.cpp         # NDT-based odometry estimation
│   ├── NDT_INC.cpp               # NDT point cloud registration
│   ├── SlamProcess.cpp           # Main SLAM orchestration
│   └── VoxelSurfelMap.cpp        # Voxelized map management
├── script/
│   └── kitti_unsynced_ros2bag.py # Utility for dataset conversion
├── test/                          # Unit tests
├── CMakeLists.txt                # Build configuration
└── package.xml                   # ROS2 package metadata
```

## Key Components

### Data Loader (SensorDataPlayer)
Reads and streams sensor data from ROS2 bags:
- Supports LiDAR (PointCloud2), IMU, and GPS data
- Provides callback-based interface for data consumption
- Thread-safe queue management for concurrent access

### Lidar Odometry (LidarOdodmetry)
Estimates motion between consecutive LiDAR scans:
- Uses NDT algorithm for point cloud registration
- Implements keyframe detection based on motion thresholds
- Maintains frame and keyframe counters

### State Estimator (ESKF)
Kalman filter for fusing motion estimates:
- Error state representation
- Handles IMU pre-integration
- Produces optimized pose estimates

### Frontend (SlamFrontEnd)
The frontend processes real-time sensor data and produces keyframes for mapping:
- Integrates Lidar Odometry, DataLoader, and ESKF
- Runs in a separate processing thread
- Emits keyframe and end-of-stream callbacks

### Map Management (VoxelizedMap)
Efficient map representation:
- Divides space into voxels
- Stores surfel information per voxel
- Supports dynamic insertion and removal

## Dependencies

### ROS2 and Core Libraries
- `rclcpp` - ROS2 C++ client library
- `sensor_msgs` - Standard ROS2 sensor message types
- `nav_msgs` - Navigation message types
- `rosbag2_cpp` - ROS2 bag reading library

### External Libraries
- `Eigen3` - Linear algebra library
- `PCL` (Point Cloud Library) - Point cloud processing
- `TBB` (Threading Building Blocks) - Parallel computing

## Building

Assuming you're in a ROS2 workspace:

```bash
# Build the package
colcon build --packages-select offline_lidar_slam

```

## Usage

### Basic Workflow

1. **Prepare Data**: Convert your dataset to ROS2 bag format
   ```bash
   # Example: Convert KITTI dataset
   python script/kitti_unsynced_ros2bag.py --dataset-path <path-to-kitti>
   ```

2. **Create SLAM Process**:
   ```cpp
   #include "SlamProcess.hpp"
   
   SlamProcess slam;
   // Process completes when bag data is exhausted
   slam.Join();  // Wait for completion
   ```

3. **Configure Parameters**: Edit [include/Config/Config.yaml](include/Config/Config.yaml) to adjust algorithm parameters for your dataset

4. **Access Results**: Map and vehicle trajectory are generated during processing in `./LIO_results`

### Supported Input Formats

- **ROS2 Bags** (.db3): Primary input format
  - Must contain LiDAR point clouds (PointCloud2 messages)
  - Optional: IMU and GPS data for sensor fusion
  
- **KITTI Dataset**: Use provided conversion script

## Configuration

Configuration is managed through [include/Config/Config.yaml](include/Config/Config.yaml). Parameters include ESKF noise settings, buffer capacities, voxel resolutions, data paths, and map generation options.

## Output

The SLAM system produces:
- **Trajectory**: Stream of poses along the robot's path
- **3D Map**: Voxelized point cloud map
- **Keyframes**: Selected frames with registered point clouds

## Limitations

- Requires offline processing (no real-time streaming)
- Depends on good initial data synchronization
- Best performance on datasets with sufficient LiDAR overlap between views
- Deterministic playback (no adaptive processing)

## Common Issues

### Bag File Reading Errors
- Ensure ROS2 bags are in .db3 format (MCAPv0 or SQLite3)
- Verify topic names match expected message types
- Check bag integrity: `ros2 bag info <bag-file>`

### Poor Odometry Registration
- Increase voxel density in point cloud filtering
- Reduce motion threshold for more keyframes
- Verify LiDAR data quality (check for NaN, Inf values)

### Memory Issues with Large Datasets
- Reduce map voxel resolution
- Implement sliding window map management
- Consider processing in segments


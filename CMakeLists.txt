cmake_minimum_required(VERSION 3.8)
project(slam_proj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ---------------- dataloader ----------------
add_library(dataloader
  src/DataLoader.cpp
)

add_library(common
  src/Common.cpp
)

add_library(NDT_lib
  src/NDT_INC.cpp
)

# ament_target_dependencies uses plain target_link_libraries internally on your system
ament_target_dependencies(dataloader
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  nav_msgs
)

# MUST be plain signature too
target_link_libraries(dataloader
  Eigen3::Eigen
  TBB::tbb
  ${PCL_LIBRARIES}
)

target_link_libraries(common
  Eigen3::Eigen
  TBB::tbb
  ${PCL_LIBRARIES}
)

add_library(LO_lib
  src/LidarOdometry.cpp
)

target_link_libraries(LO_lib
  NDT_lib
)
# ---------------- frontend ----------------
add_library(frontend
  src/ESKF.cpp
  src/FrontEnd.cpp
  src/GeoConverter.cpp
  # add other sources, but NOT DataLoader.cpp
)

add_library(slam_process
  src/VoxelSurfelMap.cpp
  src/SlamProcess.cpp
)

ament_target_dependencies(frontend
  rclcpp
  sensor_msgs
  nav_msgs
)

# MUST be plain signature too
target_link_libraries(frontend
  dataloader
  common
  NDT_lib
  LO_lib
  Eigen3::Eigen
  TBB::tbb
  ${PCL_LIBRARIES}
)

ament_target_dependencies(slam_process
  rclcpp
  sensor_msgs
  nav_msgs
)

target_link_libraries(slam_process
  frontend dataloader common Eigen3::Eigen TBB::tbb ${PCL_LIBRARIES}
)

# ---------------- Tests ----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dataloader test/TestDataLoader.cpp)
  target_link_libraries(test_dataloader dataloader)

  ament_add_gtest(test_ndt test/TestNDT.cpp)
  target_link_libraries(test_ndt NDT_lib common Eigen3::Eigen TBB::tbb ${PCL_LIBRARIES})

  ament_add_gtest(test_lo test/TestLidarOdometry.cpp)
  target_link_libraries(test_lo NDT_lib LO_lib common Eigen3::Eigen TBB::tbb ${PCL_LIBRARIES})

  ament_add_gtest(test_fe test/TestFrontEnd.cpp)
  target_link_libraries(test_fe frontend)

  ament_add_gtest(test_slam test/TestSlamProcess.cpp)
  target_link_libraries(test_slam slam_process)

endif()

ament_package()

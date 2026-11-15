cmake_minimum_required(VERSION 3.12)
project(tutorial_pcl_avoid)

# 生成 compile_commands.json，编辑器找头文件用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  mavros_msgs
  livox_ros_driver2
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mavros_msgs livox_ros_driver2
                pcl_ros pcl_conversions
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_install_python(PROGRAMS scripts/spawn_pillars.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

get_filename_component(TUTORIAL_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# student/ 中的源码由学生自行组织，仅约定 cloud_cb 的接口。
file(GLOB_RECURSE STUDENT_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/student/*.cpp")
if(NOT STUDENT_SOURCES)
  message(FATAL_ERROR "请在 tutorial_pcl_avoid/student/ 中添加自己的 C++ 源码并实现 pcl_avoid::cloud_cb，接口见 include/pcl_avoid/callback.hpp。")
endif()
add_library(pcl_avoid_detect ${STUDENT_SOURCES} src/detection_state.cpp)
target_link_libraries(pcl_avoid_detect ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_avoid_detect ${catkin_EXPORTED_TARGETS})

add_executable(offline_test src/offline_test.cpp)
target_link_libraries(offline_test pcl_avoid_detect ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_definitions(offline_test PRIVATE
  DATA_DIR="${TUTORIAL_WS_DIR}/data"
  OUT_DIR="${TUTORIAL_WS_DIR}/out")
add_dependencies(offline_test ${catkin_EXPORTED_TARGETS})

# 飞行程序：起飞 → 悬停检测 → 挑缝 → 穿过去 → 降落
add_executable(mission_node src/mission_node.cpp src/flight.cpp)
target_link_libraries(mission_node pcl_avoid_detect ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(mission_node ${catkin_EXPORTED_TARGETS})

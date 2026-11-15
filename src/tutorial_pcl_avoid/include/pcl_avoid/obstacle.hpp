#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace pcl_avoid {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

// 柱子位置与半径的近似估计；机体系，x 前、y 左、z 上，单位米。
struct Obstacle {
  float x = 0.0f;
  float y = 0.0f;
  float radius = 0.0f;
};
using ObstacleList = std::vector<Obstacle>;

}  // namespace pcl_avoid

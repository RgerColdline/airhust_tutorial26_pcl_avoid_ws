#pragma once

#include <sensor_msgs/PointCloud2.h>
#include "pcl_avoid/obstacle.hpp"

namespace pcl_avoid {

// 学生实现的回调；msg 是本帧世界系 ENU 点云。
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);

// 提交本帧检测结果。空列表表示本帧没有可用结果。
void submitDetection(const ObstacleList& obstacles);

// 可选的世界系到机体系转换工具；主控仅在悬停检测期间启用，失败时返回 false。
bool cloudToBody(const sensor_msgs::PointCloud2& msg, Cloud& output);

}  // namespace pcl_avoid

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include "pcl_avoid/callback.hpp"

namespace pcl_avoid {

// 主控运行状态，学生回调通过 callback.hpp 中的接口提交结果。
struct DetectionResult {
  ObstacleList obstacles;
  geometry_msgs::Pose pose;
  ros::Time received_at;
  unsigned long sequence = 0;
};

extern geometry_msgs::PoseStamped current_pose;
extern bool pose_received;
// 上游点云的坐标约定；学生回调始终接收世界系点云。
extern bool cloud_in_world;
extern bool detection_enabled;
extern DetectionResult detection;

// 仅用于主控输入适配。
bool cloudToWorld(const sensor_msgs::PointCloud2& msg, Cloud& output);

}  // namespace pcl_avoid

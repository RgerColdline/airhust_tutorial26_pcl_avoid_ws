#include "pcl_avoid/runtime.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>

namespace pcl_avoid {

geometry_msgs::PoseStamped current_pose;
bool pose_received = false;
bool cloud_in_world = true;
bool detection_enabled = false;
DetectionResult detection;

namespace {
bool bodyPose(Eigen::Isometry3f& transform) {
  if (!pose_received) return false;
  const auto& p = current_pose.pose.position;
  const auto& q = current_pose.pose.orientation;
  Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6f ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  transform = Eigen::Isometry3f::Identity();
  transform.linear() = rotation.normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3f(p.x, p.y, p.z);
  return true;
}

void readCloud(const sensor_msgs::PointCloud2& msg, Cloud& output) {
  pcl::fromROSMsg(msg, output);
  output.is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(output, output, indices);
}
}  // namespace

bool cloudToWorld(const sensor_msgs::PointCloud2& msg, Cloud& output) {
  Eigen::Isometry3f transform;
  if (!cloud_in_world && !bodyPose(transform)) return false;
  readCloud(msg, output);
  if (!cloud_in_world) pcl::transformPointCloud(output, output, transform.matrix());
  return true;
}

bool cloudToBody(const sensor_msgs::PointCloud2& msg, Cloud& output) {
  Eigen::Isometry3f transform;
  if (!detection_enabled || !bodyPose(transform)) return false;
  readCloud(msg, output);
  pcl::transformPointCloud(output, output, transform.inverse().matrix());
  return true;
}

void submitDetection(const ObstacleList& obstacles) {
  detection.obstacles = obstacles;
  detection.pose = current_pose.pose;
  detection.received_at = ros::Time::now();
  ++detection.sequence;
}

}  // namespace pcl_avoid

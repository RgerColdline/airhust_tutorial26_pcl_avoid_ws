// 离线调用学生回调，并比较提交的柱子检测结果。

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "pcl_avoid/runtime.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_avoid/paths.hpp"

namespace {

constexpr float kCenterTol = 0.30f;   // 允许表面点簇中心相对几何圆心的偏移
constexpr float kRadiusTol = 0.15f;   // 接受粗略尺寸估计，另检查通道判断
constexpr float kMinGap = 1.1f;       // 缝宽过多少就算能飞

struct Truth {
  float x = 0.0f;
  float y = 0.0f;
  float radius = 0.0f;
};

std::vector<Truth> loadGroundTruth(const std::string& path) {
  std::vector<Truth> gt;
  std::ifstream f(path);
  if (!f) return gt;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    Truth t;
    if (std::sscanf(line.c_str(), "%f %f %f", &t.x, &t.y, &t.radius) != 3) continue;
    gt.push_back(t);
  }
  return gt;
}

void printObstacle(const pcl_avoid::Obstacle& o) {
  std::cout << "(" << std::fixed << std::setprecision(2) << o.x << ", " << o.y
            << ") r=" << std::setprecision(3) << o.radius;
}

// 两根相邻柱子的缝：从右边柱子的左沿到左边柱子的右沿。
struct Gap {
  float center_y = 0.0f;
  float width = 0.0f;
};

std::vector<Gap> findGaps(const pcl_avoid::ObstacleList& obstacles) {
  pcl_avoid::ObstacleList sorted = obstacles;
  std::sort(sorted.begin(), sorted.end(),
            [](const pcl_avoid::Obstacle& a, const pcl_avoid::Obstacle& b) {
              return a.y < b.y;
            });
  std::vector<Gap> gaps;
  for (std::size_t i = 0; i + 1 < sorted.size(); ++i) {
    Gap g;
    g.center_y = (sorted[i].y + sorted[i].radius + sorted[i + 1].y -
                  sorted[i + 1].radius) / 2.0f;
    g.width = (sorted[i + 1].y - sorted[i + 1].radius) -
              (sorted[i].y + sorted[i].radius);
    gaps.push_back(g);
  }
  return gaps;
}

}  // namespace

int main(int argc, char** argv) {
  ros::Time::init();
  const std::string cloud_path =
      (argc > 1) ? argv[1] : dataPath("pillar_frame.pcd");
  const std::string gt_path = (argc > 2) ? argv[2] : dataPath("pillar_gt.txt");

  pcl_avoid::Cloud::Ptr cloud(new pcl_avoid::Cloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(cloud_path, *cloud) == -1) {
    std::cerr << "读不了 " << cloud_path << "\n";
    return 1;
  }
  std::cout << "点云 " << cloud_path << " —— " << cloud->size() << " 点\n";

  const std::vector<Truth> gt = loadGroundTruth(gt_path);
  if (gt.empty()) {
    std::cerr << "真值文件是空的: " << gt_path << "\n";
    return 1;
  }

  // 测试数据模拟原点上方 1 m 的悬停帧，转换成回调约定的世界系输入。
  for (auto& point : *cloud) point.z += 1.0f;
  pcl_avoid::current_pose.pose.position.z = 1.0;
  pcl_avoid::current_pose.pose.orientation.w = 1.0;
  pcl_avoid::pose_received = true;
  pcl_avoid::cloud_in_world = true;
  pcl_avoid::detection_enabled = true;
  sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud, *msg);
  msg->header.frame_id = "world";
  pcl_avoid::cloud_cb(msg);
  if (pcl_avoid::detection.sequence == 0) {
    std::cerr << "回调没有通过 submitDetection 提交本帧结果。\n";
    return 1;
  }
  const pcl_avoid::ObstacleList found = pcl_avoid::detection.obstacles;

  for (const auto& obstacle : found) {
    if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
        !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0f) {
      std::cerr << "检测结果含无效坐标或半径。\n";
      return 1;
    }
  }

  std::cout << "\n真值 " << gt.size() << " 根，你找到 " << found.size() << " 根\n";
  std::cout << std::left << std::setw(8) << "真值" << std::setw(26) << ""
            << std::setw(10) << "位置偏差" << "半径误差\n";

  bool ok = (found.size() == gt.size());
  std::vector<bool> used(found.size(), false);
  for (const auto& t : gt) {
    int best = -1;
    float best_d = 1e9f;
    for (std::size_t i = 0; i < found.size(); ++i) {
      if (used[i]) continue;
      const float d = std::hypot(found[i].x - t.x, found[i].y - t.y);
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    std::cout << "(" << std::fixed << std::setprecision(2) << t.x << ", " << t.y
              << ") r=" << std::setprecision(3) << t.radius;
    if (best < 0) {
      std::cout << "   没找到\n";
      ok = false;
      continue;
    }
    used[best] = true;
    const float r_err = std::fabs(found[best].radius - t.radius);
    const bool within_tolerance = best_d <= kCenterTol && r_err <= kRadiusTol;
    if (!within_tolerance) ok = false;
    std::cout << "    " << std::setprecision(3) << best_d << " m      " << r_err
              << " m" << (within_tolerance ? "" : "  超出允许误差") << "\n";
  }
  if (found.size() > gt.size()) {
    std::cout << "多出来的障碍物:\n";
    for (std::size_t i = 0; i < found.size(); ++i) {
      if (used[i]) continue;
      std::cout << "  ";
      printObstacle(found[i]);
      std::cout << "\n";
    }
    ok = false;
  }

  const std::vector<Gap> gaps = findGaps(found);
  std::cout << "\n你找到的柱子之间的缝:\n";
  for (const auto& g : gaps) {
    std::cout << "  y=" << std::setprecision(2) << g.center_y << " 宽 "
              << std::setprecision(2) << g.width
              << (g.width >= kMinGap ? "  ← 能过" : "  （太窄）") << "\n";
  }

  pcl_avoid::ObstacleList truth_obstacles;
  for (const auto& t : gt) {
    pcl_avoid::Obstacle obstacle;
    obstacle.x = t.x;
    obstacle.y = t.y;
    obstacle.radius = t.radius;
    truth_obstacles.push_back(obstacle);
  }
  const auto truth_gaps = findGaps(truth_obstacles);
  if (gaps.size() != truth_gaps.size()) ok = false;
  for (std::size_t i = 0; i < std::min(gaps.size(), truth_gaps.size()); ++i) {
    if ((gaps[i].width >= kMinGap) != (truth_gaps[i].width >= kMinGap)) {
      std::cout << "第 " << i + 1 << " 个通道的通行判断不正确。\n";
      ok = false;
    }
  }

  std::cout << (ok ? "\n通过\n" : "\n没通过\n");
  return ok ? 0 : 1;
}

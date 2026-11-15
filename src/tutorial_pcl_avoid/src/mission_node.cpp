#include "pcl_avoid/runtime.hpp"
#include "pcl_avoid/flight.hpp"

#include <livox_ros_driver2/CustomMsg.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <clocale>
#include <cstdio>
#include <string>

using namespace pcl_avoid;

// 输入由回调更新，任务在单线程主循环中推进。
mavros_msgs::State current_state;
bool scene_ready = false;
void scene_cb(const std_msgs::Bool::ConstPtr& msg) { scene_ready = msg->data; }

void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_pose = *msg;
  pose_received = true;
}

// 上游可以是世界系或局部点云，学生回调统一接收世界系 PointCloud2。
void pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (cloud_in_world) {
    cloud_cb(msg);
    return;
  }
  static Cloud world;
  if (!cloudToWorld(*msg, world)) return;
  sensor_msgs::PointCloud2::Ptr converted(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(world, *converted);
  converted->header = msg->header;
  converted->header.frame_id = current_pose.header.frame_id.empty() ? "map" : current_pose.header.frame_id;
  cloud_cb(converted);
}

// 原始 Livox 消息也转成 PointCloud2，学生只需实现一种回调接口。
void livox_cb(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
  static Cloud cloud;
  cloud.clear();
  cloud.reserve(msg->points.size());
  for (const auto& p : msg->points) {
    pcl::PointXYZI point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.reflectivity;
    cloud.push_back(point);
  }
  sensor_msgs::PointCloud2::Ptr converted(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud, *converted);
  converted->header = msg->header;
  pointcloud_cb(converted);
}

struct Passage {
  double y = 0.0;
  double before_x = 0.0;
  double after_x = 0.0;
};

// 将检测时刻的机体系结果还原到世界系，再计算通道。
bool choose_passage(const DetectionResult& result, Passage& passage) {
  if (result.obstacles.size() != 4) return false;
  const auto& pose = result.pose;
  Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) return false;
  rotation.normalize();
  ObstacleList world;
  for (const auto& ob : result.obstacles) {
    if (!std::isfinite(ob.x) || !std::isfinite(ob.y) || !std::isfinite(ob.radius) ||
        ob.radius <= 0.0f) return false;
    const Eigen::Vector3d p = rotation * Eigen::Vector3d(ob.x, ob.y, 0.0) +
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Obstacle w = ob;
    w.x = p.x();
    w.y = p.y();
    world.push_back(w);
  }
  std::sort(world.begin(), world.end(), [](const Obstacle& a, const Obstacle& b) {
    return a.y < b.y;
  });
  bool found = false;
  double best_distance = 1e9;
  double min_x = world.front().x, max_x = min_x;
  for (const auto& ob : world) {
    min_x = std::min(min_x, static_cast<double>(ob.x - ob.radius));
    max_x = std::max(max_x, static_cast<double>(ob.x + ob.radius));
  }
  for (std::size_t i = 0; i + 1 < world.size(); ++i) {
    const double lower = world[i].y + world[i].radius;
    const double upper = world[i + 1].y - world[i + 1].radius;
    if (upper - lower < flight.min_gap) continue;
    const double y = (upper + lower) / 2.0;
    if (std::fabs(y - flight.goal_y) >= best_distance) continue;
    passage = {y, min_x - 0.8, max_x + 0.8};
    best_distance = std::fabs(y - flight.goal_y);
    found = true;
  }
  return found;
}

void print_param() {
  ROS_INFO("takeoff_height=%.2f, goal=(%.2f, %.2f), min_gap=%.2f, max_speed=%.2f",
           flight.takeoff_height, flight.goal_x, flight.goal_y, flight.min_gap, flight.max_speed);
}

enum Mission { WAIT_READY, WAIT_START, TAKEOFF, DETECT, ALIGN, CROSS, GOAL, LAND, DONE, FAILED };

int main(int argc, char** argv) {
  std::setlocale(LC_ALL, "");
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string topic, msg_type, frame;
  pnh.param<std::string>("cloud_topic", topic, "/cloud_registered");
  pnh.param<std::string>("msg_type", msg_type, "pointcloud2");
  pnh.param<std::string>("cloud_frame", frame, "world");
  pnh.param("takeoff_height", flight.takeoff_height, 1.0);
  pnh.param("goal_x", flight.goal_x, 8.0);
  pnh.param("goal_y", flight.goal_y, 0.0);
  pnh.param("min_gap", flight.min_gap, 1.1);
  pnh.param("max_speed", flight.max_speed, 0.5);
  if ((frame != "world" && frame != "body") ||
      (msg_type != "pointcloud2" && msg_type != "custom") ||
      !std::isfinite(flight.max_speed) || flight.max_speed <= 0.0 ||
      !std::isfinite(flight.takeoff_height) || flight.takeoff_height <= 0.0 ||
      !std::isfinite(flight.goal_x) || !std::isfinite(flight.goal_y) ||
      !std::isfinite(flight.min_gap) || flight.min_gap <= 0.0) {
    ROS_FATAL("点云配置或飞行参数无效。");
    return 1;
  }
  cloud_in_world = frame == "world";

  auto scene_sub = nh.subscribe("/pcl_avoid/scene_ready", 1, scene_cb);
  auto state_sub = nh.subscribe("mavros/state", 10, state_cb);
  auto pose_sub = nh.subscribe("mavros/local_position/pose", 10, pose_cb);
  ros::Subscriber cloud_sub = msg_type == "custom"
      ? nh.subscribe(topic, 1, livox_cb) : nh.subscribe(topic, 1, pointcloud_cb);
  auto setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  auto arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  auto mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  print_param();

  ros::Rate rate(20);
  Mission mission = WAIT_READY;
  ros::Time phase_start = ros::Time::now(), last_request;
  unsigned long last_sequence = 0;
  Passage passage;
  double start_x = 0.0, start_y = 0.0;
  auto advance = [&](Mission next) {
    mission = next;
    phase_start = ros::Time::now();
    ROS_INFO("mission=%d", static_cast<int>(mission));
  };
  auto fail = [&]() {
    const auto& p = current_pose.pose.position;
    hold_position(p.x, p.y, p.z, current_yaw());
    ROS_ERROR("任务中止，保持当前位置。请检查检测结果与飞控状态。");
    advance(FAILED);
  };
  auto request_mode = [&](const std::string& mode) {
    if (ros::Time::now() - last_request < ros::Duration(1.0)) return;
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    if (!mode_client.call(srv) || !srv.response.mode_sent)
      ROS_WARN_THROTTLE(2.0, "飞控未接受模式切换请求。");
    last_request = ros::Time::now();
  };

  while (ros::ok()) {
    detection_enabled = mission == DETECT;
    ros::spinOnce();
    const auto now = ros::Time::now();
    const auto& p = current_pose.pose.position;
    if (mission >= TAKEOFF && mission <= GOAL &&
        (!current_state.connected || !current_state.armed || current_state.mode != "OFFBOARD")) fail();
    if (mission >= TAKEOFF && mission <= GOAL && now - phase_start > ros::Duration(60.0)) fail();

    switch (mission) {
      case WAIT_READY:
        if (current_state.connected && pose_received && scene_ready) {
          init_setpoint();
          start_x = p.x;
          start_y = p.y;
          advance(WAIT_START);
        }
        break;
      case WAIT_START:
        // 先连续发送设定点，再请求 OFFBOARD 与解锁。
        if (now - phase_start < ros::Duration(1.0)) break;
        if (current_state.mode != "OFFBOARD") request_mode("OFFBOARD");
        else if (!current_state.armed && now - last_request > ros::Duration(1.0)) {
          mavros_msgs::CommandBool srv;
          srv.request.value = true;
          if (!arming_client.call(srv) || !srv.response.success)
            ROS_WARN_THROTTLE(2.0, "飞控未接受解锁请求。");
          last_request = now;
        } else if (current_state.armed) advance(TAKEOFF);
        break;
      case TAKEOFF:
        if (mission_pos_cruise(start_x, start_y, flight.takeoff_height)) advance(DETECT);
        break;
      case DETECT:
        mission_pos_cruise(start_x, start_y, flight.takeoff_height);
        if (now - phase_start > ros::Duration(15.0)) {
          if (detection.sequence == 0) ROS_ERROR("检测超时：尚未收到回调提交的结果，请检查话题和回调。");
          else ROS_ERROR("检测超时：结果中没有完整、可通行的柱子布局。");
          fail();
          break;
        }
        if (now - phase_start < ros::Duration(1.0) || detection.sequence == last_sequence) break;
        last_sequence = detection.sequence;
        if (detection.received_at < phase_start + ros::Duration(1.0) ||
            now - detection.received_at > ros::Duration(1.0)) break;
        ROS_INFO_THROTTLE(1.0, "本帧识别 %zu 根柱子。", detection.obstacles.size());
        if (choose_passage(detection, passage)) {
          ROS_INFO("通道 y=%.2f，入口 x=%.2f，出口 x=%.2f", passage.y, passage.before_x, passage.after_x);
          advance(ALIGN);
        } else ROS_INFO_THROTTLE(1.0, "等待完整、可通行的检测结果。");
        break;
      case ALIGN:
        if (mission_pos_cruise(passage.before_x, passage.y, flight.takeoff_height)) advance(CROSS);
        break;
      case CROSS:
        if (mission_pos_cruise(passage.after_x, passage.y, flight.takeoff_height)) advance(GOAL);
        break;
      case GOAL:
        if (mission_pos_cruise(flight.goal_x, flight.goal_y, flight.takeoff_height)) advance(LAND);
        break;
      case LAND:
        request_mode("AUTO.LAND");
        if (!current_state.armed) {
          ROS_INFO("已落地，任务完成。");
          advance(DONE);
        }
        break;
      case DONE:
        return 0;
      case FAILED:
        break;
    }
    if (pose_received) {
      setpoint_raw.header.stamp = ros::Time::now();
      setpoint_pub.publish(setpoint_raw);
    }
    rate.sleep();
  }
  return 0;
}

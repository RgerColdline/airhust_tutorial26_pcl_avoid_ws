#pragma once

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

namespace pcl_avoid {

struct FlightConfig {
  double takeoff_height = 1.0;
  double goal_x = 8.0;
  double goal_y = 0.0;
  double min_gap = 1.1;
  double max_speed = 0.5;
  double arrival_error = 0.15;
};

extern FlightConfig flight;
extern mavros_msgs::PositionTarget setpoint_raw;

void init_setpoint();
// 目标为 MAVROS 本地 ENU 坐标，yaw 单位为弧度；到达后返回 true。
bool mission_pos_cruise(double x, double y, double z, double yaw = 0.0);
void hold_position(double x, double y, double z, double yaw);
double current_yaw();

}  // namespace pcl_avoid

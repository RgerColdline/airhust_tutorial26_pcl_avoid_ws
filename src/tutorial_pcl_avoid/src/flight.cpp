#include "pcl_avoid/flight.hpp"
#include "pcl_avoid/runtime.hpp"

#include <algorithm>
#include <cmath>

namespace pcl_avoid {

FlightConfig flight;
mavros_msgs::PositionTarget setpoint_raw;
ros::Time command_time;

double current_yaw() {
  const auto& q = current_pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void init_setpoint() {
  setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                          mavros_msgs::PositionTarget::IGNORE_VY |
                          mavros_msgs::PositionTarget::IGNORE_VZ |
                          mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ |
                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  const auto& p = current_pose.pose.position;
  hold_position(p.x, p.y, p.z, current_yaw());
}

void hold_position(double x, double y, double z, double yaw) {
  setpoint_raw.position.x = x;
  setpoint_raw.position.y = y;
  setpoint_raw.position.z = z;
  setpoint_raw.yaw = yaw;
  command_time = ros::Time::now();
}

bool mission_pos_cruise(double x, double y, double z, double yaw) {
  const auto now = ros::Time::now();
  const double dt = std::clamp((now - command_time).toSec(), 0.0, 0.1);
  command_time = now;
  const double dx = x - setpoint_raw.position.x;
  const double dy = y - setpoint_raw.position.y;
  const double dz = z - setpoint_raw.position.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double scale = distance > 1e-6 ? std::min(1.0, flight.max_speed * dt / distance) : 1.0;
  setpoint_raw.position.x += scale * dx;
  setpoint_raw.position.y += scale * dy;
  setpoint_raw.position.z += scale * dz;
  setpoint_raw.yaw = yaw;

  const auto& p = current_pose.pose.position;
  const double error = std::sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y) +
                                 (p.z - z) * (p.z - z));
  const double yaw_error = std::remainder(current_yaw() - yaw, 2.0 * std::acos(-1.0));
  return error < flight.arrival_error && std::fabs(yaw_error) < 0.1;
}

}  // namespace pcl_avoid

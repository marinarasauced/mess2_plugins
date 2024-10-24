#ifndef MESS2_PLUGINS__ROTATION_HPP_
#define MESS2_PLUGINS__ROTATION_HPP_

#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include </usr/include/armadillo>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "mess2_msgs/msg/euler_angles.hpp"
#include "mess2_msgs/msg/vertex.hpp"

namespace mess2_plugins {

geometry_msgs::msg::Quaternion normalize_quat(geometry_msgs::msg::Quaternion quat);

geometry_msgs::msg::Quaternion average_two_quats(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2);

mess2_msgs::msg::EulerAngles convert_quat_to_eul(geometry_msgs::msg::Quaternion quat);

geometry_msgs::msg::Quaternion convert_eul_to_quat(mess2_msgs::msg::EulerAngles eul);

geometry_msgs::msg::Quaternion invert_quat(geometry_msgs::msg::Quaternion quat);

geometry_msgs::msg::Quaternion multiply_two_quats(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2);

double wrap_to_pi(double angle);

double get_angle_between_edges(mess2_msgs::msg::Vertex vertex0, mess2_msgs::msg::Vertex vertex1, mess2_msgs::msg::Vertex vertex2);

}  // namespace mess2_plugins

#endif  // MESS2_PLUGINS__ROTATION_HPP_
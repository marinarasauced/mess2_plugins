#ifndef MESS2_PLUGINS__CALIBRATION_HPP_
#define MESS2_PLUGINS__CALIBRATION_HPP_

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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "mess2_msgs/msg/euler_angles.hpp"
#include "mess2_plugins/rotation.hpp"

namespace mess2_plugins {

geometry_msgs::msg::Transform update_measurement(const geometry_msgs::msg::Transform meas1, const geometry_msgs::msg::Transform meas2, const int64_t weight);

geometry_msgs::msg::Quaternion get_vicon_calibration(const geometry_msgs::msg::Transform meas1, const geometry_msgs::msg::Transform meas2);

}  // namespace mess2_plugins

#endif  // MESS2_PLUGINS__CALIBRATION_HPP_
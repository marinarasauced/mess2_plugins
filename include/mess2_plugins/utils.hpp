#ifndef MESS2_PLUGINS__UTILS_HPP_
#define MESS2_PLUGINS__UTILS_HPP_

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

namespace mess2_plugins {

std::string get_cmd_vel_topic();

std::string get_vicon_topic(std::string agent_name);

}  // namespace mess2_plugins

#endif  // MESS2_PLUGINS__UTILS_HPP_
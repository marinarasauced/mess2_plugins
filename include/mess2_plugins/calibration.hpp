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
#include "mess2_plugins/common.hpp"

using Transform = geometry_msgs::msg::Transform;
using Quaternion = geometry_msgs::msg::Quaternion;

namespace mess2_plugins {


    /**
     * @brief recursively incorporates new measurements into a weighted average of already sampled measurements.
     * 
     * @param meas1 the current measurement average.
     * @param meas2 the new measurement to be added to meas1.
     * @param weight the weight of meas2 depending on the number of measurements averaged into meas1.
     * @return the new Transform ROS2 object instance containing the updated measurement average.
     */
    Transform update_measurement(const Transform meas1, const Transform meas2, const int64_t weight);


    /**
     * @brief find the difference quaternion necessary for the driver to correctly calibrate object orientations.
     * 
     * @param meas1
     * @param meas2
     * @return the difference quaternion.
     */
    Quaternion get_vicon_calibration(const Transform meas1, const Transform meas2);

}

#endif  // MESS2_PLUGINS__CALIBRATION_HPP_

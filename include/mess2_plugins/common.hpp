#ifndef MESS2_PLUGINS_COMMON_HPP
#define MESS2_PLUGINS_COMMON_HPP

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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mess2_msgs/msg/euler_angles.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;
using Quaternion = geometry_msgs::msg::Quaternion;
using EulerAngles = mess2_msgs::msg::EulerAngles;

namespace mess2_plugins {


    /**
     * @brief normalizes a ROS2 Quaternion object instance.
     * 
     * @param q1 the quaternion to be normalized.
     * @return the equivalent of q1 with a magnitude of one.
     */
    Quaternion normalize_quaternion(const Quaternion &q1);


    /**
     * @brief averages two ROS2 Quaternion object instances.
     * 
     * @param q1 the first quaternion to be averaged.
     * @param q2 the second quaternion to be averaged.
     * @return the average of q1 and q2.
     */
    Quaternion average_quaternions(const Quaternion &q1, const Quaternion &q2);


    /**
     * @brief inverts a ROS2 Quaternion object instance.
     * 
     * @param q1 the quaternion to be inverted.
     * @return the inverse quaternion of q1.
     */
    Quaternion invert_quaternion(const Quaternion &q1);


    /**
     * @brief converts a ROS2 Quaternion object instance to a ROS2 mess2 EulerAngles object instance.
     * 
     * @param q1 the quaternion to be converted to euler angles.
     * @return the euler angle equivalent of q1.
     */
    EulerAngles convert_quaternion_to_euler_angles(const Quaternion &q1);


    /**
     * @brief converts a ROS2 mess2 EulerAngles object instance to a ROS2 Quaternion object instance.
     * 
     * @param e1 the euler angles to be converted to a quaternion.
     * @return the quaternion equivalent to e1.
     */
    Quaternion convert_euler_angles_to_quaternion(const EulerAngles &e1);


    /**
     * @brief multiples two ROS2 Quaternion object instances.
     * 
     * @param q1 the first quaternion to be multiplied.
     * @param q2 the second quaternion to be multiplied.
     * @return the product of q1 and q2.
     */
    Quaternion multiply_quaternions(const Quaternion &q1, const Quaternion &q2);


    /**
     * @brief wraps an angle to [-pi, pi] bounds.
     * 
     * @param angle the angle to wrap to [-pi, pi].
     * @return the equivalent of angle in [-pi, pi].
     */
    double wrap_angle_to_pi(double angle);

}

#endif  // MESS2_PLUGINS_COMMON_HPP

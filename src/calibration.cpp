
#include "mess2_plugins/calibration.hpp"

namespace mess2_plugins {

    geometry_msgs::msg::Transform update_measurement(const geometry_msgs::msg::Transform avg_, const geometry_msgs::msg::Transform new_, const int64_t weight)
    {
        geometry_msgs::msg::Transform result;
        result.translation.x = (avg_.translation.x * weight + new_.translation.x) / (weight + 1);
        result.translation.y = (avg_.translation.y * weight + new_.translation.y) / (weight + 1);
        result.translation.z = (avg_.translation.z * weight + new_.translation.z) / (weight + 1);
        result.rotation.x = (avg_.rotation.x * weight + new_.rotation.x) / (weight + 1);
        result.rotation.y = (avg_.rotation.y * weight + new_.rotation.y) / (weight + 1);
        result.rotation.z = (avg_.rotation.z * weight + new_.rotation.z) / (weight + 1);
        result.rotation.w = (avg_.rotation.w * weight + new_.rotation.w) / (weight + 1);
        return result;
    }

    geometry_msgs::msg::Quaternion get_vicon_calibration(const geometry_msgs::msg::Transform meas1, const geometry_msgs::msg::Transform meas2)
    {
        double yaw = std::atan2(
            meas2.translation.y - meas1.translation.y,
            meas2.translation.x - meas1.translation.x
        );

        mess2_msgs::msg::EulerAngles euler;
        euler.roll = 0.0;
        euler.pitch = 0.0;
        euler.yaw = yaw;

        auto quat1 = meas1.rotation;
        auto quat2 = meas2.rotation;

        auto temp1 = mess2_plugins::convert_eul_to_quat(euler);
        auto temp2 = mess2_plugins::multiply_two_quats(quat1, quat2);
        auto temp3 = mess2_plugins::invert_quat(temp2);

        auto quat_diff = mess2_plugins::multiply_two_quats(temp3, temp1);
        quat_diff = mess2_plugins::normalize_quat(quat_diff);
        return quat_diff;
    }

}
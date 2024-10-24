
#include "mess2_plugins/rotation.hpp"

namespace mess2_plugins {

    geometry_msgs::msg::Quaternion normalize_quat(geometry_msgs::msg::Quaternion quat) 
    {
        geometry_msgs::msg::Quaternion quat_norm;
        double magnitude = std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
        quat_norm.x = quat.x / magnitude;
        quat_norm.y = quat.y / magnitude;
        quat_norm.z = quat.z / magnitude;
        quat_norm.w = quat.w / magnitude;
        return quat_norm;
    }

    geometry_msgs::msg::Quaternion average_two_quats(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2) 
    {
        geometry_msgs::msg::Quaternion quat_avg;
        geometry_msgs::msg::Quaternion quat_avg_norm;
        geometry_msgs::msg::Quaternion quat1_norm;
        geometry_msgs::msg::Quaternion quat2_norm;
        quat1_norm = normalize_quat(quat1);
        quat2_norm = normalize_quat(quat2);
        double quat_dot = quat1_norm.x * quat2_norm.x + quat1_norm.y * quat2_norm.y + quat1_norm.z * quat2_norm.z + quat1_norm.w * quat2_norm.w;
        double sign = (quat_dot < 0.0) ? -1.0 : 1.0;
        quat_avg.x = (quat1.x + sign * quat2.x) / 2;
        quat_avg.y = (quat1.y + sign * quat2.y) / 2;
        quat_avg.z = (quat1.z + sign * quat2.z) / 2;
        quat_avg.w = (quat1.w + sign * quat2.w) / 2;
        quat_avg_norm = normalize_quat(quat_avg);
        return quat_avg_norm;
    }

    mess2_msgs::msg::EulerAngles convert_quat_to_eul(geometry_msgs::msg::Quaternion quat) 
    {
        mess2_msgs::msg::EulerAngles eul;
        eul.roll = std::atan2((2 * quat.w * quat.x + quat.y * quat.z), 1 - 2 * (quat.x * quat.x + quat.y * quat.y));
        double sign = 2 * (quat.w * quat.y - quat.x - quat.z);
        if (std::abs(sign) >= 1.0) {
            eul.pitch = std::copysign(0.5 * M_PI, sign);
        } else {
            eul.pitch = -0.5 * M_PI * std::asin(sign);
        }
        eul.yaw = std::atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y * quat.y + quat.z * quat.z));
        return eul;
    }

    geometry_msgs::msg::Quaternion convert_eul_to_quat(mess2_msgs::msg::EulerAngles eul) 
    {
        geometry_msgs::msg::Quaternion quat;
        double cx = std::cos(0.5 * eul.roll);
        double sx = std::sin(0.5 * eul.roll);
        double cy = std::cos(0.5 * eul.pitch);
        double sy = std::sin(0.5 * eul.pitch);
        double cz = std::cos(0.5 * eul.yaw);
        double sz = std::sin(0.5 * eul.yaw);
        quat.x = sx * cy * cz - cx * sy * sz;
        quat.y = cx * sy * cz + sx * cy * sz;
        quat.z = cx * cy * sz - sx * sy * cz;
        quat.w = cx * cy * cz + sx * sy * sz;
        return quat;
    }

    geometry_msgs::msg::Quaternion invert_quat(geometry_msgs::msg::Quaternion quat) 
    {
        geometry_msgs::msg::Quaternion quat_inv;
        quat_inv.x = -quat.x;
        quat_inv.y = -quat.y;
        quat_inv.z = -quat.z;
        quat_inv.w = quat.w;
        return quat_inv;
    }

    geometry_msgs::msg::Quaternion multiply_two_quats(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2) 
    {
        geometry_msgs::msg::Quaternion quat_product;
        quat_product.x = quat1.w * quat2.x + quat1.x * quat2.w + quat1.y * quat2.z - quat1.z * quat2.y;
        quat_product.y = quat1.w * quat2.y - quat1.x * quat2.z + quat1.y * quat2.w + quat1.z * quat2.x;
        quat_product.z = quat1.w * quat2.z + quat1.x * quat2.y - quat1.y * quat2.x + quat1.z * quat2.w;
        quat_product.w = quat1.w * quat2.w - quat1.x * quat2.x - quat1.y * quat2.y - quat1.z * quat2.z;
        return quat_product;
    }

    double wrap_to_pi(double angle) 
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle <= -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    double get_angle_between_edges(mess2_msgs::msg::Vertex vertex0, mess2_msgs::msg::Vertex vertex1, mess2_msgs::msg::Vertex vertex2)
    {
        double dx1_10 = vertex1.position.x - vertex0.position.x;
        double dx2_10 = vertex1.position.y - vertex0.position.y;
        double dx1_21 = vertex2.position.x - vertex1.position.x;
        double dx2_21 = vertex2.position.y - vertex1.position.y;
        double dot = dx1_10 * dx1_21 + dx2_10 * dx2_21;
        double mag_10 = std::sqrt(dx1_10 * dx1_10 + dx2_10 * dx2_10);
        double mag_21 = std::sqrt(dx1_21 * dx1_21 + dx2_21 * dx2_21);
        double theta_inv = dot / (mag_10 * mag_21);
        theta_inv = std::max(-1.0, std::min(1.0, theta_inv));
        double theta = std::acos(theta_inv);
        return theta;
    }
}
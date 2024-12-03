
#include "mess2_plugins/common.hpp"

namespace mess2_plugins {


    Quaternion normalize_quaternion(const Quaternion &q1)
    {
        Quaternion norm;
        double mag = std::sqrt(q1.x * q1.x + q1.y * q1.y + q1.z * q1.z + q1.w * q1.w);
        norm.x = q1.x / mag;
        norm.y = q1.y / mag;
        norm.z = q1.z / mag;
        norm.w = q1.w / mag;
        return norm;
    }


    Quaternion average_quaternions(const Quaternion &q1, const Quaternion &q2)
    {
        Quaternion avg;
        auto q1_ = normalize_quaternion(q1);
        auto q2_ = normalize_quaternion(q2);
        double dot = q1_.x * q2_.x + q1_.y * q2_.y + q1_.z * q2_.z + q1_.w * q2_.w;
        double sign = (dot < 0.0) ? -1.0 : 1.0;
        avg.x = (q1.x + sign * q2.x) / 2;
        avg.y = (q1.y + sign * q2.y) / 2;
        avg.z = (q1.z + sign * q2.z) / 2;
        avg.w = (q1.w + sign * q2.w) / 2;
        return avg;
    }


    Quaternion invert_quaternion(const Quaternion &q1)
    {
        Quaternion inv;
        inv.x = -q1.x;
        inv.y = -q1.y;
        inv.z = -q1.z;
        inv.w = -q1.w;
        return inv;
    }

    EulerAngles convert_quaternion_to_euler_angles(const Quaternion &q1)
    {
        EulerAngles e1;
        e1.roll = std::atan2((2 * q1.w * q1.x + q1.y * q1.z), 1 - 2 * (q1.x * q1.x + q1.y * q1.y));
        double sign = 2 * (q1.w * q1.y - q1.x - q1.z);
        if (std::abs(sign) >= 1.0) {
            e1.pitch = std::copysign(0.5 * M_PI, sign);
        } else {
            e1.pitch = -0.5 * M_PI * std::asin(sign);
        }
        e1.yaw = std::atan2(2 * (q1.w * q1.z + q1.x * q1.y), 1 - 2 * (q1.y * q1.y + q1.z * q1.z));
        return e1;
    }


    Quaternion convert_euler_angles_to_quaternion(const EulerAngles &e1)
    {
        Quaternion q1;
        double cx = std::cos(0.5 * e1.roll);
        double sx = std::sin(0.5 * e1.roll);
        double cy = std::cos(0.5 * e1.pitch);
        double sy = std::sin(0.5 * e1.pitch);
        double cz = std::cos(0.5 * e1.yaw);
        double sz = std::sin(0.5 * e1.yaw);
        q1.x = sx * cy * cz - cx * sy * sz;
        q1.y = cx * sy * cz + sx * cy * sz;
        q1.z = cx * cy * sz - sx * sy * cz;
        q1.w = cx * cy * cz + sx * sy * sz;
        return q1;
    }


    Quaternion multiply_quaternions(const Quaternion &q1, const Quaternion &q2)
    {
        Quaternion prod;
        prod.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        prod.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        prod.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
        prod.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        return prod;
    }


    double wrap_angle_to_pi(double angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle <= -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

}

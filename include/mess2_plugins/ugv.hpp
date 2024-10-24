
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
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

#include "mess2_msgs/msg/ugv_state.hpp"
#include "mess2_plugins/rotation.hpp"

namespace mess2_plugins
{
    double get_max_u_lin(const std::string model, const double speed);
    double get_max_u_ang(const std::string model, const double speed);

    mess2_msgs::msg::UGVState get_error_from_line(const mess2_msgs::msg::UGVState global, const mess2_msgs::msg::UGVState init, const mess2_msgs::msg::UGVState trgt);
    double get_error_from_heading(const double global,const double trgt);
}
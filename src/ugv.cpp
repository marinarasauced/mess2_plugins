
#include "mess2_plugins/ugv.hpp"

namespace mess2_plugins {

    double get_max_u_lin(const std::string model, const double speed)
    {
        if (model == "burger")
        {
            return 0.22 * speed;
        }
        else if (model == "wafflepi")
        {
            return 0.26 * speed;
        }
        else
        {
            return 0.22 * speed;
        }
    }

    double get_max_u_ang(const std::string model, const double speed)
    {
        if (model == "burger")
        {
            return 2.84 * speed;
        }
        else if (model == "wafflepi")
        {
            return 1.82 * speed;
        }
        else
        {
            return 1.82 * speed;
        }
    }

    mess2_msgs::msg::UGVState get_error_from_line(const mess2_msgs::msg::UGVState global, const mess2_msgs::msg::UGVState init, const mess2_msgs::msg::UGVState trgt)
    {
        arma::vec A = {trgt.state.x - init.state.x, trgt.state.y - init.state.y};
        arma::vec B = {trgt.state.x - global.state.x, trgt.state.y - global.state.y};
        arma::vec C = {global.state.x - init.state.x, global.state.y - init.state.y};
        double a = arma::norm(A);
        double b = arma::norm(B);
        double alpha = std::acos(arma::dot(A, B) / (a * b));
        double beta = std::atan2(C(1), C(0));
        double theta = std::atan2(A(1), A(0));
        
        mess2_msgs::msg::UGVState error;
        error.state.x = b * std::cos(alpha);
        error.state.y = b * std::sin(alpha) * std::copysign(1.0, beta - theta);
        error.state.theta = wrap_to_pi(global.state.theta - theta);
        return error;
    }

    double get_error_from_heading(const double global,const double trgt)
    {
        double error = wrap_to_pi(global - trgt);
        return error;
    }
}

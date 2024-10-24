
#include "mess2_plugins/utils.hpp"

namespace mess2_plugins {

    std::string get_cmd_vel_topic()
    {
        std::string topic = "cmd_vel";
        return topic;
    }

    std::string get_vicon_topic(std::string agent_name) 
    {
        std::string topic = "/vicon/" + agent_name + "/" + agent_name;
        return topic;
    }
}
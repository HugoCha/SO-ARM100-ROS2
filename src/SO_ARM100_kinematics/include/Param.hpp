#pragma once

#include <rclcpp/node.hpp>
#include <string>

namespace SOArm100::Kinematics::Params
{

template<typename T>
struct Param
{
    std::string_view name;
    T default_value;

    T Get(const rclcpp::Node::SharedPtr& node,
          const std::string& group_name) const
    {
        const std::string full_name =
            group_name + "." + std::string{name};

        node->declare_parameter<T>(full_name, default_value);

        T value;
        node->get_parameter(full_name, value);

        return value;
    }
};
}
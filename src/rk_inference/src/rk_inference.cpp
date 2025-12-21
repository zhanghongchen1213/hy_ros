#include "rk_inference.h"
#include <rclcpp_components/register_node_macro.hpp>

RkInference::RkInference(const rclcpp::NodeOptions & options)
: Node("rk_inference", options)
{
    RCLCPP_INFO(this->get_logger(), "RkInference initialized");
}

RkInference::~RkInference()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(RkInference)

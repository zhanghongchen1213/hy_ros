#include "audio_rkllm.h"

class HelloWorldNode : public rclcpp::Node
{
public:
    HelloWorldNode()
        : Node("hello_world_node")
    {
        // 创建定时器，每500ms触发一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&HelloWorldNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello world!");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#ifndef AUDIO_RKLLM_H_
#define AUDIO_RKLLM_H_

#include "rclcpp/rclcpp.hpp"
#include "rkllm.h"  // 引入 RKLLM 头文件

// 简单的 RKLLM 测试类
class RKLLMTestNode : public rclcpp::Node
{
public:
    RKLLMTestNode();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    bool rkllm_initialized_;
};


#endif

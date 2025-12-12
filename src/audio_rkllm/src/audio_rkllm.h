#ifndef AUDIO_RKLLM_H_
#define AUDIO_RKLLM_H_

#include "rclcpp/rclcpp.hpp"
#include "rkllm.h"
#include <opencv2/opencv.hpp>
#include <string>

// 简单的 RKLLM 测试类
class RKLLMTestNode : public rclcpp::Node
{
public:
    RKLLMTestNode();

private:
    void timer_callback();
    
    // 拍摄图像并保存，返回保存的文件路径
    std::string capture_image();

    rclcpp::TimerBase::SharedPtr timer_;
    bool rkllm_initialized_;
    
    // 图像保存路径
    std::string image_save_path_ = "/tmp/capture.jpg";
    
    // 摄像头翻转参数
    bool camera_flip_;
};


#endif

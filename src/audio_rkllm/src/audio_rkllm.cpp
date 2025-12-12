#include "audio_rkllm.h"

RKLLMTestNode::RKLLMTestNode()
    : Node("rkllm_test_node"), rkllm_initialized_(false)
{
    // 声明并读取 camera_flip 参数
    this->declare_parameter<bool>("camera_flip", true);
    this->get_parameter("camera_flip", camera_flip_);
    RCLCPP_INFO(this->get_logger(), "摄像头翻转参数 (camera_flip): %s", camera_flip_ ? "True (正向)" : "False (反向/旋转180度)");

    // 创建定时器，每 1000ms 触发一次，用于周期性检查状态
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&RKLLMTestNode::timer_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "RKLLM Test Node Started.");

    // 尝试调用 RKLLM 的初始化函数（仅测试链接，不传入真实参数）
    // 注意：rkllm_init 通常需要真实的模型路径和配置，这里传入 nullptr 预期会失败
    // 但我们的目的是验证符号 'rkllm_init' 是否能被成功链接和调用。
    LLMHandle handle = nullptr;
    RKLLMParam param;
    // 简单初始化结构体，避免未定义行为
    param.model_path = ""; 
    
    RCLCPP_INFO(this->get_logger(), "Attempting to call rkllm_init to verify linking...");
    
    // 调用 rkllm_init。即使失败，只要不报 "undefined reference"，说明链接成功。
    // 修复：rkllm_init 的第二个参数定义为 RKLLMParam 类型（值传递或引用），而非指针。
    // 根据报错 'could not convert ... to RKLLMParam'，说明函数签名期望的是 RKLLMParam 结构体本身，而不是其指针。
    int ret = rkllm_init(&handle, param, nullptr);
    
    if (ret == 0) {
        RCLCPP_INFO(this->get_logger(), "rkllm_init returned 0 (Success? Unexpected for empty param).");
    } else {
        // 这是预期结果，因为参数为空。关键是我们成功调用了它！
        RCLCPP_INFO(this->get_logger(), "rkllm_init called successfully (returned %d, as expected for empty params). Linking verified!", ret);
    }
    
    // 如果 handle 被赋值了（虽然不太可能），记得释放
    if (handle != nullptr) {
        rkllm_destroy(handle);
    }
}

void RKLLMTestNode::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Node is running... (RKLLM symbols are available)");
    
    // 模拟：假设从 ASR 收到了文本 "帮我拍摄一张照片"
    std::string mock_asr_text = "帮我拍摄一张照片";
    
    if (mock_asr_text.find("拍摄") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "检测到关键词'拍摄'，正在调用摄像头...");
        std::string img_path = capture_image();
        if (!img_path.empty()) {
            RCLCPP_INFO(this->get_logger(), "图像已保存至: %s", img_path.c_str());
            // TODO: 这里将 img_path 传给 RKLLM 进行多模态推理
        } else {
            RCLCPP_WARN(this->get_logger(), "拍摄失败");
        }
    }
}

std::string RKLLMTestNode::capture_image()
{
    // 打开默认摄像头 (索引 0)
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开摄像头!");
        return "";
    }

    // 设置分辨率 (可选，根据硬件支持调整)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    cv::Mat frame;
    // 尝试读取几帧以让摄像头自动曝光稳定
    for(int i = 0; i < 5; i++) {
        cap >> frame;
    }

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "无法获取图像帧!");
        return "";
    }

    // 根据参数翻转图像
    if (!camera_flip_) {
        // 如果设置为 False，进行 180 度旋转 (水平+垂直翻转)
        cv::flip(frame, frame, -1);
        RCLCPP_INFO(this->get_logger(), "已对图像进行180度旋转");
    }

    // 保存图像
    if (cv::imwrite(image_save_path_, frame)) {
        return image_save_path_;
    } else {
        RCLCPP_ERROR(this->get_logger(), "保存图像失败!");
        return "";
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RKLLMTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

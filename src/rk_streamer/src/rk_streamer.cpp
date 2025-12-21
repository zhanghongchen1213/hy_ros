#include "rk_streamer.h"
#include <rclcpp_components/register_node_macro.hpp>

RkStreamer::RkStreamer(const rclcpp::NodeOptions & options)
: Node("rk_streamer", options)
{
    // 声明参数
    this->declare_parameter("width", 1920);
    this->declare_parameter("height", 1080);
    this->declare_parameter("fps", 30);
    this->declare_parameter("sub_topic", "/yolo/image_infer");
    
    // GStreamer 管道配置
    // 注意：输入格式设置为 NV12，以便直接透传或由硬件编码器处理
    std::string default_pipeline = "appsrc ! video/x-raw,format=NV12,width=1920,height=1080 ! mpph264enc ! rtspclientsink location=rtsp://127.0.0.1:8554/live";
    this->declare_parameter("pipeline", default_pipeline);

    // 获取参数
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    std::string topic = this->get_parameter("sub_topic").as_string();
    pipeline_ = this->get_parameter("pipeline").as_string();

    RCLCPP_INFO(this->get_logger(), "初始化 RkStreamer, 订阅: %s", topic.c_str());
    RCLCPP_INFO(this->get_logger(), "GStreamer 管道: %s", pipeline_.c_str());

    // 创建订阅者
    // 使用 image_transport 支持压缩传输（虽然这里我们直接处理 raw NV12）
    sub_ = image_transport::create_subscription(
        this, 
        topic, 
        std::bind(&RkStreamer::image_callback, this, std::placeholders::_1), 
        "raw", 
        rmw_qos_profile_sensor_data
    );

    // 尝试初始化 VideoWriter
    if (width_ > 0 && height_ > 0) {
        init_video_writer();
    }
}

RkStreamer::~RkStreamer()
{
    if (writer_ && writer_->isOpened()) {
        writer_->release();
    }
}

void RkStreamer::init_video_writer()
{
    if (is_initialized_) return;

    try {
        writer_ = std::make_unique<cv::VideoWriter>(
            pipeline_, 
            cv::CAP_GSTREAMER, 
            0, // fourcc ignored for GStreamer
            fps_, 
            cv::Size(width_, height_), 
            true // isColor
        );

        if (!writer_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开 VideoWriter (GStreamer管道可能错误)");
        } else {
            RCLCPP_INFO(this->get_logger(), "VideoWriter 已打开 (%dx%d@%d)", width_, height_, fps_);
            is_initialized_ = true;
        }
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV 异常: %s", e.what());
    }
}

void RkStreamer::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    // 如果未初始化且参数为0，则使用第一帧的尺寸
    if (!is_initialized_) {
        if (width_ <= 0) width_ = msg->width;
        if (height_ <= 0) height_ = msg->height;
        init_video_writer();
    }

    if (!is_initialized_ || !writer_ || !writer_->isOpened()) {
        return; // 初始化失败或未就绪
    }

    // 关键：处理 NV12 格式数据
    // 假设输入 msg->encoding 为 "nv12" 且数据已经是 NV12 格式
    // 此时 msg->data 包含 Y + UV 数据
    // NV12 高度 = H + H/2 = 1.5 * H
    // OpenCV 中可以用 CV_8UC1 来包装这块内存，高度设为 1.5倍
    
    // 简单校验一下数据大小
    size_t expected_size = width_ * height_ * 3 / 2;
    if (msg->data.size() < expected_size) {
        RCLCPP_WARN(this->get_logger(), "数据大小不足 NV12: expected %zu, got %zu", expected_size, msg->data.size());
        return;
    }

    try {
        // 创建 cv::Mat 包装 msg 数据，不拷贝
        // 注意：const_cast 是必须的，因为 cv::Mat 构造函数不接受 const void*，但只要我们不修改它且 writer 只读即可
        cv::Mat nv12_img(height_ + height_ / 2, width_, CV_8UC1, (void*)&msg->data[0]);

        // 写入 GStreamer 管道 (OpenCV 可能会做一次拷贝到底层 Buffer，但这已经是极小的开销)
        writer_->write(nv12_img);
        
        // 调试日志 (每30帧)
        static int count = 0;
        if (++count % 30 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "已推流 %d 帧", count);
        }

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV write 异常: %s", e.what());
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RkStreamer)

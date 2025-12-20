#include "rk_camera.h"
#include <rclcpp_components/register_node_macro.hpp>

RkCamera::RkCamera(const rclcpp::NodeOptions & options)
: Node("rk_camera", options)
{
    // 声明参数
    this->declare_parameter("device_id", 0);
    this->declare_parameter("width", 1920);
    this->declare_parameter("height", 1080);
    this->declare_parameter("framerate", 30);
    this->declare_parameter("image_topic", "/camera/image_raw");
    this->declare_parameter("debug_fps", false);

    // 获取参数
    int device_id = this->get_parameter("device_id").as_int();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    int framerate = this->get_parameter("framerate").as_int();
    std::string topic_name = this->get_parameter("image_topic").as_string();
    debug_fps_ = this->get_parameter("debug_fps").as_bool();

    // 参数回调
    params_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & param : parameters) {
                if (param.get_name() == "debug_fps") {
                    debug_fps_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "调试帧率日志已%s", debug_fps_ ? "开启" : "关闭");
                }
            }
            return result;
        }
    );

    RCLCPP_INFO(this->get_logger(), "初始化 RkCamera，设备=/dev/video%d, %dx%d@%d",
        device_id, width_, height_, framerate);

    // 创建发布者
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

    // 初始化 GStreamer
    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    // 构建 Pipeline
    // v4l2src -> videoconvert -> video/x-raw,format=NV12 -> appsink
    std::ostringstream ss;
    ss << "v4l2src device=/dev/video" << device_id << " ! "
       << "videoconvert ! "
       << "video/x-raw,format=NV12,width=" << width_ << ",height=" << height_ << ",framerate=" << framerate << "/1 ! "
       << "appsink name=sink emit-signals=true max-buffers=1 drop=true";
    
    std::string pipe_str = ss.str();
    RCLCPP_INFO(this->get_logger(), "GStreamer 管道: %s", pipe_str.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipe_str.c_str(), &error);
    
    if (error) {
        RCLCPP_ERROR(this->get_logger(), "GStreamer 管道解析失败: %s", error->message);
        g_error_free(error);
        return;
    }

    if (!pipeline_) {
        RCLCPP_ERROR(this->get_logger(), "无法创建 GStreamer 管道");
        return;
    }

    // 获取 sink 元素并设置回调
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (sink) {
        g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample_stub), this);
        gst_object_unref(sink);
    } else {
        RCLCPP_ERROR(this->get_logger(), "无法获取 sink 元素");
        return;
    }

    // 启动 Pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(this->get_logger(), "无法启动管道 (设置状态为 PLAYING 失败)");
    } else {
        RCLCPP_INFO(this->get_logger(), "GStreamer 管道已启动");
    }

    // Create timer to check bus
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            if (!pipeline_) return;
            GstBus *bus = gst_element_get_bus(pipeline_);
            if (bus) {
                GstMessage *msg = gst_bus_pop_filtered(bus, 
                    static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING));
                if (msg) {
                    GError *err;
                    gchar *debug_info;
                    switch (GST_MESSAGE_TYPE(msg)) {
                        case GST_MESSAGE_ERROR:
                            gst_message_parse_error(msg, &err, &debug_info);
                            RCLCPP_ERROR(this->get_logger(), "GStreamer 错误: %s", err->message);
                            RCLCPP_ERROR(this->get_logger(), "调试信息: %s", debug_info ? debug_info : "无");
                            g_clear_error(&err);
                            g_free(debug_info);
                            break;
                        case GST_MESSAGE_EOS:
                            RCLCPP_INFO(this->get_logger(), "GStreamer 流结束 (EOS)");
                            break;
                        case GST_MESSAGE_WARNING:
                            gst_message_parse_warning(msg, &err, &debug_info);
                            RCLCPP_WARN(this->get_logger(), "GStreamer 警告: %s", err->message);
                            RCLCPP_WARN(this->get_logger(), "调试信息: %s", debug_info ? debug_info : "无");
                            g_clear_error(&err);
                            g_free(debug_info);
                            break;
                        default:
                            break;
                    }
                    gst_message_unref(msg);
                }
                gst_object_unref(bus);
            }
        }
    );
}

RkCamera::~RkCamera()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

GstFlowReturn RkCamera::on_new_sample_stub(GstElement* sink, gpointer user_data)
{
    return static_cast<RkCamera*>(user_data)->on_new_sample(sink);
}

GstFlowReturn RkCamera::on_new_sample(GstElement* sink)
{
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {
        RCLCPP_WARN(this->get_logger(), "无法获取样本 (pull sample 失败)");
        return GST_FLOW_ERROR;
    }

    // Log every 30 frames to confirm data flow
    static int frame_count = 0;
    if (++frame_count % 30 == 0) {
        if (debug_fps_) {
            RCLCPP_INFO(this->get_logger(), "正在处理第 %d 帧", frame_count);
        }
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        // 创建消息 (使用 UniquePtr)
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        msg->height = height_;
        msg->width = width_;
        msg->encoding = "nv12"; // 自定义 encoding
        msg->is_bigendian = false;
        msg->step = width_;     // NV12 Y平面步长 = 宽度

        // 数据拷贝 (GStreamer Buffer -> ROS Msg Vector)
        // 这一步拷贝是必要的，除非使用自定义 Allocator
        msg->data.resize(map.size);
        memcpy(&msg->data[0], map.data, map.size);

        // 发布消息 (移动所有权，实现进程内零拷贝)
        pub_->publish(std::move(msg));
        
        gst_buffer_unmap(buffer, &map);
    }
    
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// 注册组件 (移动到文件末尾)
RCLCPP_COMPONENTS_REGISTER_NODE(RkCamera)

#ifndef RK_STREAMER_H
#define RK_STREAMER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

class RkStreamer : public rclcpp::Node {
public:
    explicit RkStreamer(const rclcpp::NodeOptions & options);
    ~RkStreamer();

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void init_video_writer();

    image_transport::Subscriber sub_;
    std::unique_ptr<cv::VideoWriter> writer_;
    
    // Parameters
    int width_;
    int height_;
    int fps_;
    std::string pipeline_;
    bool is_initialized_ = false;
};

#endif // RK_STREAMER_H

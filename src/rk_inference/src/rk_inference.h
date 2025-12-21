#ifndef RK_INFERENCE_H_
#define RK_INFERENCE_H_

#include <rclcpp/rclcpp.hpp>

class RkInference : public rclcpp::Node {
public:
    explicit RkInference(const rclcpp::NodeOptions & options);
    ~RkInference();

private:
};

#endif // RK_INFERENCE_H_

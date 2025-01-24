#ifndef MPCC_CONTROLLER_H
#define MPCC_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mpcc_control/state.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mpcc {
class MPCC : public rclcpp::Node
{
public:
    MPCC();

private:
    void timer_callback();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    State car_state_;
    Input car_input_;

    bool get_state_;
};
}

#endif //MPCC_CONTROLLER_H
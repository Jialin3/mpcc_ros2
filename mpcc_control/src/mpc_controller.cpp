#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mpcc {
class MPCC : public rclcpp::Node
{
public:
    MPCC(): Node("mpcc_node") {
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MPCC::timer_callback, this));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MPCC::odom_callback, this, std::placeholders::_1));
    }
private:
    void timer_callback() {}
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {}
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mpcc::MPCC>());
    rclcpp::shutdown();
    return 0;
}
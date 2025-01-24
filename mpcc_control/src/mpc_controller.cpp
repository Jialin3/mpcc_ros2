#include "mpcc_control/mpc_controller.hpp"

namespace mpcc {

MPCC::MPCC(): Node("mpcc_node") {
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MPCC::timer_callback, this));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MPCC::odom_callback, this, std::placeholders::_1));
    car_state_.setZeros();
    car_input_.setZero();
     
    RCLCPP_INFO(this->get_logger(), "牛逼");
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    get_state_ = false;
}

void MPCC::timer_callback() {}

void MPCC::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    try {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        double x = transform_stamped.transform.translation.x;
        double y = transform_stamped.transform.translation.y;

        tf2::Quaternion quat(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        
        car_state_.x = x;
        car_state_.y = y;
        car_state_.vx = vx;
        car_state_.vy = vy;
        car_state_.phi = yaw;
        get_state_ = true;
    } catch (tf2::TransformException &ex) {
        get_state_ = false;
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
}

}

int main(int argc, char *argv[]) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建 MPCC 节点
    auto mpcc_node = std::make_shared<mpcc::MPCC>();

    // 运行节点
    rclcpp::spin(mpcc_node);

    // 关闭 ROS 2
    rclcpp::shutdown();

    return 0;
}

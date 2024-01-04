#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/highgui/highgui.hpp>

  

class ImuOdomDisplayNode : public rclcpp::Node

{

public:

ImuOdomDisplayNode() : Node("imodo_display_node")

{

imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(

"/imu/data_raw", 10,

[this](sensor_msgs::msg::Imu::SharedPtr msg) {

imuCallback(msg);

});

  

odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(

"/odom", 10,

[this](nav_msgs::msg::Odometry::SharedPtr msg) {

odomCallback(msg);

});

  

if (!imu_sub_ || !odom_sub_) {

RCLCPP_ERROR(get_logger(), "Failed to create subscriptions.");

rclcpp::shutdown();

}

}

  

private:

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)

{

RCLCPP_INFO(get_logger(), "IMU: [%f, %f, %f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

}

  

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)

{

RCLCPP_INFO(get_logger(), "Odometry: Position: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

}

  

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

};

  

int main(int argc, char *argv[])

{

rclcpp::init(argc, argv);

auto node = std::make_shared<ImuOdomDisplayNode>();

rclcpp::spin(node);

rclcpp::shutdown();

return 0;

}
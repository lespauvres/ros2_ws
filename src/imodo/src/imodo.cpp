#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ImuOdomDisplayNode : public rclcpp::Node
{
public:
    ImuOdomDisplayNode() : Node("imodo_display_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 10,
            std::bind(&ImuOdomDisplayNode::imuCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ImuOdomDisplayNode::odomCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "IMU: [%f, %f, %f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Odometry: Position: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdomDisplayNode>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraDisplayNode : public rclcpp::Node
{
public:
    CameraDisplayNode() : Node("camera_display_node")
    {
        // 订阅颜色相机数据
        color_sub_ = image_transport::create_subscription(
            this, "/camera/color/image_raw",
            std::bind(&CameraDisplayNode::colorCallback, this, std::placeholders::_1),
            "raw");

        // 订阅深度相机数据
        depth_sub_ = image_transport::create_subscription(
            this, "/camera/depth/image_rect_raw",
            std::bind(&CameraDisplayNode::depthCallback, this, std::placeholders::_1),
            "raw");
    }

private:
    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imshow("Color Image", color_img);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            cv::Mat depth_img = cv_bridge::toCvShare(msg)->image;
            cv::imshow("Depth Image", depth_img);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s'.", msg->encoding.c_str());
        }
    }

    image_transport::Subscriber color_sub_;
    image_transport::Subscriber depth_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDisplayNode>());
    rclcpp::shutdown();
    return 0;
}
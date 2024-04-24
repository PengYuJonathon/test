#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "/opt/ros/foxy/include/cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        subscriber_ = image_transport::create_subscription(
            this, "camera/image",
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1),
            "raw");

        publisher_ = image_transport::create_publisher(this, "camera/image_blended_gray");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ImageSubscriber::timer_callback1, this));
    }

private:
    Mat gray_image;
    Mat image;
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cvtColor(image, gray_image, COLOR_BGR2GRAY);
    }

    void timer_callback1()
    {

        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image");
            return;
        }

        if (gray_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load gray_image");
            return;
        }

        auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_image).toImageMsg();
        publisher_.publish(message);

        imshow("Subscribe GRAY",gray_image);
        waitKey(1);
    }

    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

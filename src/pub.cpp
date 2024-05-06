#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "/opt/ros/foxy/include/cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using std::cout;
using std::endl;

const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;
Mat src1, src2, dst;

static void on_trackbar(int, void *)
{
  alpha = (double)alpha_slider / alpha_slider_max;
  beta = (1.0 - alpha);
  addWeighted(src1, alpha, src2, beta, 0.0, dst);
  imshow("Linear Blend", dst);
}

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher() : Node("image_publisher")
  {
    publisher_ = image_transport::create_publisher(this, "camera/image");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {

    on_trackbar(alpha_slider, 0);

    auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dst).toImageMsg();
    publisher_.publish(message);

    waitKey(10);
  }
  image_transport::Publisher publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  src1 = imread("/home/yupeng/ros2_ws/src/trackbar/src/linux.png");
  src2 = imread("/home/yupeng/ros2_ws/src/trackbar/src/windows.png");

  if (src1.empty())
  {
    cout << "Error loading src1" << endl;
  }
  if (src2.empty())
  {
    cout << "Error loading src2" << endl;
  }

  alpha_slider = 0;
  namedWindow("Linear Blend", WINDOW_AUTOSIZE);

  char TrackbarName[50];
  sprintf(TrackbarName, "Alpha x %d", alpha_slider_max);
  createTrackbar(TrackbarName, "Linear Blend", &alpha_slider, alpha_slider_max, on_trackbar);

  auto node = std::make_shared<ImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

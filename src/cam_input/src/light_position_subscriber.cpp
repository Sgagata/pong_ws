// this part of the code was written to check whether the light position detection works properly

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp"


class LightPositionSubscriber : public rclcpp::Node
{
public:
    LightPositionSubscriber() : Node("light_position_subscriber"), count_(0)
    {
        //create subscribers for the sent light positions
        left_light_subscriber_ = create_subscription<geometry_msgs::msg::Point>("left_light_position", 10, 
        std::bind(&LightPositionSubscriber::left_light_callback, this, std::placeholders::_1));

        right_light_subscriber_ = create_subscription<geometry_msgs::msg::Point>("right_light_position", 10, 
        std::bind(&LightPositionSubscriber::right_light_callback, this, std::placeholders::_1));

        frame_width_ = 640;
        frame_height_ = 480;

        // Create window for display with black background
        cv::namedWindow("Light Positions", cv::WINDOW_NORMAL);
        cv::resizeWindow("Light Positions", frame_width_, frame_height_);
        cv::imshow("Light Positions", black_image_);
        //time to process
        cv::waitKey(1);
    }
private:

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr left_light_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr right_light_subscriber_;
    int frame_width_;
    int frame_height_;
    cv::Mat black_image_ = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
    size_t count_;


    //draw a green point where the COG for left side of the screen is
    void left_light_callback(const geometry_msgs::msg::Point::SharedPtr msg){
        //scale the possition so that it mathces the display window size
        cv::Point left_point(static_cast<int>(msg->x*frame_width_/2), static_cast<int>(msg->y*frame_height_));
        cv::circle(black_image_, left_point, 5, cv::Scalar(0, 255, 0, 255), -1);
        cv::imshow("Light Positions", black_image_);
        cv::waitKey(1);
    }


    void right_light_callback(const geometry_msgs::msg::Point::SharedPtr msg){
        //works the same as for the left light but the 0.5 of width is added so that the point is displayed on a right hand size
        cv::Point right_point(static_cast<int>(msg->x*frame_width_/2 + frame_width_/2), static_cast<int>(msg->y*frame_height_));
        cv::circle(black_image_, right_point, 5, cv::Scalar(255, 0, 0, 255), -1);
        cv::imshow("Light Positions", black_image_);
        cv::waitKey(1);
    }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPositionSubscriber>());
  rclcpp::shutdown();
  return 0;
}


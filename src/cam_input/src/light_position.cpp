#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "geometry_msgs/msg/point_stamped.hpp"


class LightPosition : public rclcpp::Node
{
public:
    LightPosition() : Node("light_position") 
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("light_position", 10);
        camera_ = cv::VideoCapture(0); // use the default camera

        // Check if the camera was opened successfully
        if (!camera_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
        }

        // Set the resolution
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LightPosition::analyze_frame, this));

    }

private:

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    cv::VideoCapture camera_;
    rclcpp::TimerBase::SharedPtr timer_;


    void analyze_frame(){

        //get ROS2 image to open CV format
        cv::Mat image;
        camera_ >> image;

        //to gray scale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        int lower_threshold = 100;
        int upper_threshold = 255;

        //apply threshold
        cv::threshold(image, image, lower_threshold, upper_threshold, cv::THRESH_BINARY);

        // //find the center of gravity by looking at the average position of the pixels
        // cv::Mat bright_pixels;
        // cv::findNonZero(image, bright_pixels);

        // double sum_x = 0;
        // double sum_y = 0;
        // for (int i = 0; i < bright_pixels.total(); i++) {
        //     sum_x += bright_pixels.at<cv::Point>(i).x;
        //     sum_y += bright_pixels.at<cv::Point>(i).y;
        // }

        // double cx = sum_x / bright_pixels.total();
        // double cy = sum_y / bright_pixels.total();


        // Compute center of gravity version 2
        cv::Moments moments = cv::moments(image, true);
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;

        // Publish light position
        auto position_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        //set timestamp to current time
        position_msg->header.stamp = this->now();
        position_msg->header.frame_id = "camera_link";
        position_msg->point.x = cx;
        position_msg->point.y = cy;
        //transfer pointer position_msg to publish()
        publisher_->publish(std::move(position_msg));

    }   

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPosition>());
  rclcpp::shutdown();
  return 0;
}
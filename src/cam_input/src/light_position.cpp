//==============================================================
//  Filename : light_position.cpp
//  Authors : Franka van Jaarsveld, Agata Sowa
//  Group : 22
//  License: N.A. or open source license like LGPL
//  Description : Detremines the center of gravity of lightest pixesl in the image
//==============================================================

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.hpp"

class LightPosition : public rclcpp::Node
{
public:
    LightPosition() : Node("light_position"), count_(0)
    {
        // create publsiher for the light position on the left and right side of the screen
        left_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("left_light_position", 10);
        right_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("right_light_position", 10);

        // use the ready cam2image node
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&LightPosition::image_callback, this, std::placeholders::_1));
    }

private:
    // define all the publisher and subscibers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr left_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr right_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    size_t count_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        // get ROS2 image to open CV format
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_image->image;
        // cam2image does not flip the obtained image
        // left side in reality is right side in cam2image hence
        cv::flip(image, image, 1);

        // to gray scale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        // threhold for dectecting light pixels
        int lower_threshold = 250;
        int upper_threshold = 255;

        int image_width = image.cols;
        int image_height = image.rows;

        // splitting the image into left and right hand side
        cv::Mat image_left = image(cv::Rect(0, 0, image_width / 2, image_height));
        cv::Mat image_right = image(cv::Rect(image_width / 2, 0, image_width / 2, image_height));

        // apply threshold
        cv::threshold(image_left, image_left, lower_threshold, upper_threshold, cv::THRESH_BINARY);
        cv::threshold(image_right, image_right, lower_threshold, upper_threshold, cv::THRESH_BINARY);

        cv::imshow("Left Thresholded Image", image_left);
        cv::waitKey(1);

        // display right thresholded image
        cv::imshow("Right Thresholded Image", image_right);
        cv::waitKey(1);

        // there are two options check later which one is better

        // Option 1: find the center of gravity by looking at the average position of the pixels
        cv::Mat bright_pixels_left;
        cv::findNonZero(image_left, bright_pixels_left);

        double sum_x = 0;
        double sum_y = 0;
        for (int i = 0; i < bright_pixels_left.total(); i++)
        {
            sum_x += bright_pixels_left.at<cv::Point>(i).x;
            sum_y += bright_pixels_left.at<cv::Point>(i).y;
        }

        double cx_left = sum_x / bright_pixels_left.total();
        double cy_left = sum_y / bright_pixels_left.total();

        // Option2: Compute center of gravity using moments

        cv::Moments moments_right = cv::moments(image_right, true);
        double cx_right = moments_right.m10 / moments_right.m00;
        double cy_right = moments_right.m01 / moments_right.m00;

        // to map the output from 0 to 1, so its more robust towards changing window size
        cx_left = cx_left / (image_width / 2);
        cy_left = cy_left / image_height;

        cx_right = cx_right / (image_width / 2);
        cy_right = cy_right / image_height;

        // Publish left light position
        auto left_position_msg = std::make_unique<geometry_msgs::msg::Point>();
        // set timestamp to current time
        left_position_msg->x = cx_left;
        left_position_msg->y = cy_left;
        // transfer pointer position_msg to publish()
        left_publisher_->publish(std::move(left_position_msg));
        RCLCPP_INFO(this->get_logger(), "Left_Height '%d'", cy_left);

        // Publish right light position
        auto right_position_msg = std::make_unique<geometry_msgs::msg::Point>();
        right_position_msg->x = cx_right;
        right_position_msg->y = cy_right;
        right_publisher_->publish(std::move(right_position_msg));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightPosition>());
    rclcpp::shutdown();
    return 0;
}
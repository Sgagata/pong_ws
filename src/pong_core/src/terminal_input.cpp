//get the input from the terminal to move the bar
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <string>

class TerminalInputPublisher : public rclcpp::Node
{
public:
  TerminalInputPublisher() : Node("terminal_input_publisher")
  {
    // Create a publisher with a topic "input" and a queue size of 10
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("terminal_input", 10);

    // Start a timer that triggers the callback every 100 milliseconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TerminalInputPublisher::timer_callback, this));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    // Read input from terminal for y-coordinate
    std::string input;
    std::cout << "Enter y-coordinate: ";
    std::getline(std::cin, input);

    // Convert input to a double
    double y;
    try {
      y = std::stod(input);
    } catch (const std::exception& e) {
      // Invalid input, ignore
      return;
    }

    auto message = geometry_msgs::msg::Point();
    message.x = 1.0;
    message.y = y;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerminalInputPublisher>());
  rclcpp::shutdown();
  return 0;
}

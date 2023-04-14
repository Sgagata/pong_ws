#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/windowsize.hpp"


class WindowSizePublisher : public rclcpp::Node
{
public:
    WindowSizePublisher() : Node("window_size_publisher"), count_(0)
    {
      //create publisher for the window size
      window_size_publisher_ = this->create_publisher<custom_messages::msg::Windowsize>("window_size", 10);

      //create a timer so that this message gets publsihed every 500ms
      //TODO find a way for this to be a bit smarter
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&WindowSizePublisher::timer_callback, this));
      
    }

private:

  rclcpp::Publisher<custom_messages::msg::Windowsize>::SharedPtr window_size_publisher_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;


  void timer_callback()
  {
    //publsih only if there is subscriber connected
    //for the final version count how many other nodes will subscibe, publish only then
    // if (window_size_publisher_->get_subscription_count() > -1) {
      auto message = custom_messages::msg::Windowsize();                     

      message.height = 480; 
      message.width = 640;

      window_size_publisher_->publish(message);
    // }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WindowSizePublisher>());
  rclcpp::shutdown();
  return 0;
}
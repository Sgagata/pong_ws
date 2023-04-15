//No need for this class there is server
#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/windowsize.hpp"
#include "std_msgs/msg/string.hpp"



class WindowSizePublisher : public rclcpp::Node
{
public:
    WindowSizePublisher() : Node("window_size_publisher"), subscribers_count_(0)
    {
      //create publisher for the window size
      window_size_publisher_ = this->create_publisher<custom_messages::msg::Windowsize>("window_size", 10);

      //declare the window size as public parameters
      // this->declare_parameter("window_height", 480);
      // this->declare_parameter("window_width", 640);

      // window_height_ = this->get_parameter("window_height").get_value<int>();
      // window_width_ = this->get_parameter("window_width").get_value<int>();


      //TODO find a way for this to be a bit smarter
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WindowSizePublisher::timer_callback, this));

      // int test_public_ = 10;


    }

    //declare getters
    // int getWindowHeight() const { return window_height_; }
    // int getWindowWidth() const { return window_width_; }

private:

  rclcpp::Publisher<custom_messages::msg::Windowsize>::SharedPtr window_size_publisher_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_checker_;
  int subscribers_count_;
  rclcpp::TimerBase::SharedPtr timer_;
  // int window_height_;
  // int window_width_;


  void timer_callback()
  {
    //based on the ros2 humble tutorial
    //logging the params and the changing them to original values prevent changes from terminal
    // std::int16_t window_width = this->get_parameter("window_width").as_int();
    // std::int16_t window_height = this->get_parameter("window_height").as_int();

    // RCLCPP_INFO(this->get_logger(), "Hello %d!", window_height);

    // std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("window_width", 640), rclcpp::Parameter("window_height", 480)};
    // this->set_parameters(all_new_parameters);


    //publsih only if there is subscriber connected
    //for the final version count how many other nodes will subscibe, publish only then
    // if (window_size_publisher_->get_subscription_count() > -1) {
      auto message = custom_messages::msg::Windowsize();                     

      message.height = 480; 
      message.width = 640;

      window_size_publisher_->publish(message);
    // }

        // Check the number of subscribers to the topic
    // if (subscribers_count_ > 0)
    // {
    //   // If there is at least one subscriber, create a publisher and publish a message
    //   if (!window_size_publisher_)
    //   {
    //     window_size_publisher_ = this->create_publisher<custom_messages::msg::Windowsize>("window_size", 10);
    //     RCLCPP_INFO(this->get_logger(), "Publisher created.");
    //   }

    //   auto message = custom_messages::msg::Windowsize();
    //   message.height = 480; 
    //   message.width = 640;

    //   window_size_publisher_->publish(message);

    // } else {
    //   // If there are no subscribers, reset the publisher
    //   window_size_publisher_.reset();
    //   RCLCPP_INFO(this->get_logger(), "Publisher reset.");
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
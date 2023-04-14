#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/gamestate.hpp"


class GameStatePublisher : public rclcpp::Node
{
public:
    GameStatePublisher() : Node("Game_state_publisher"), count_(0)
    {
        game_state_publisher_ = this->create_publisher<custom_messages::msg::Gamestate>("game_state", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&GameStatePublisher::timer_callback, this));
    
    }


private:
  rclcpp::Publisher<custom_messages::msg::Gamestate>::SharedPtr game_state_publisher_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    //publish the state of the game
    auto message = custom_messages::msg::Gamestate();                     

    message.state = 3;

    game_state_publisher_->publish(message);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
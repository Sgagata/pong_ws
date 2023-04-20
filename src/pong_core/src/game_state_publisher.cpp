#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/gamestate.hpp"
#include "custom_messages/msg/score.hpp"
#include "custom_messages/msg/scorevalue.hpp"

class GameStatePublisher : public rclcpp::Node
{
public:
  GameStatePublisher() : Node("game_state_publisher"), count_(0)
  {
    // create a subsciber for the score
    score_subscriber_ = this->create_subscription<custom_messages::msg::Score>("score", 10,
                                                                               std::bind(&GameStatePublisher::score_callback, this, std::placeholders::_1));
    // create publisher for game state
    game_state_publisher_ = this->create_publisher<custom_messages::msg::Gamestate>("game_state", 10);

    score_publisher_ = this->create_publisher<custom_messages::msg::Scorevalue>("score_value", 10);

    score_left_ = 0;
    score_right_ = 0;
  }

private:
  rclcpp::Publisher<custom_messages::msg::Gamestate>::SharedPtr game_state_publisher_;
  rclcpp::Publisher<custom_messages::msg::Scorevalue>::SharedPtr score_publisher_;

  rclcpp::Subscription<custom_messages::msg::Score>::SharedPtr score_subscriber_;
  size_t count_;
  // rclcpp::TimerBase::SharedPtr timer_;
  int score_left_;
  int score_right_;
  int game_state_;

  // change only when new goal is scored and only publsih a
  void score_callback(const custom_messages::msg::Score::SharedPtr score)
  {
    RCLCPP_INFO(this->get_logger(), "RECEIVED %d", score->score);

    if (score->score == 'L')
    {
      score_left_ += 1;
      game_state_ = 2;
    }
    else if (score->score == 'R')
    {
      score_right_ += 1;
      game_state_ = 3;
    }
    if (score_right_ == 10 || score_left_ == 10)
    {
      game_state_ = 0;
      // resets the score
      score_right_ = 0;
      score_left_ = 0;
    }
    RCLCPP_INFO(this->get_logger(), "The score %d, : %d", score_left_, score_right_);

    auto game_state_message = custom_messages::msg::Gamestate();

    game_state_message.state = game_state_;

    auto score_value_message = custom_messages::msg::Scorevalue();
    score_value_message.left_score = score_left_;
    score_value_message.right_score = score_right_;

    game_state_publisher_->publish(game_state_message);
    score_publisher_->publish(score_value_message);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
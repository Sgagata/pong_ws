#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/windowsize.hpp"
#include "custom_messages/msg/gamestate.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"


class RightBarPosition : public rclcpp::Node
{
public:
    RightBarPosition() : Node("right_bar_position"), count_(0)
    {
        //create subscriber for the position of the bar given by light
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("right_light_position", 10, 
                std::bind(&RightBarPosition::position_callback, this, std::placeholders::_1));

        window_size_subsciber_ = this->create_subscription<custom_messages::msg::Windowsize>("window_size", 10,
                std::bind(&RightBarPosition::window_callback, this, std::placeholders::_1));

        game_state_subscriber_ = this->create_subscription<custom_messages::msg::Gamestate>("Game_state", 10,
                std::bind(&RightBarPosition::game_state_callback, this, std::placeholders::_1));

        position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("right_bar_pos", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RightBarPosition::timer_callback, this));

        bar_spacing_ = 0.02;
        bar_velocity_ = 5;

    }

private:
    size_t count_;
    //for subscribing
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Windowsize>::SharedPtr window_size_subsciber_;
    rclcpp::Subscription<custom_messages::msg::Gamestate>::SharedPtr game_state_subscriber_;
    //for publishing
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;


    // Member variables to store current position and window size information
    int window_width_; // Width of the window
    int window_height_; // Height of the window
    float bar_spacing_; //Spacing between the bar and the walls
    int bar_velocity_; //set what is the speed of the bar
    int current_x_; // Current y-coordinate of the bar
    int current_y_;// Current y-coordinate of the bar
    int game_state_; //for reading the state of the game



    void window_callback(const custom_messages::msg::Windowsize::SharedPtr size){
        //update the width and the height based on received info
        window_width_ = size->width;
        window_height_ = size->height;
        current_x_= window_width_ - window_width_*bar_spacing_; // the x position of the bar stay the same through the game

        // if its the start of the game set the bars in the middle of the screen
        if (game_state_ == 3){
            current_y_ = window_height_/2;
        }

        RCLCPP_INFO(this->get_logger(), "Width '%d'", window_width_);
        RCLCPP_INFO(this->get_logger(), "Curr_x '%d'", current_x_);
        RCLCPP_INFO(this->get_logger(), "new '%d'", current_x_);


    }

    void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
        //new_y intialized as current on will be changed through this void
        int new_y = current_y_;
        //steering if the light is above half move up otherwise move down - the light point comes between 0 and 1
        //maybe implement increase in spee based on distance
        if (msg->point.y > 0.5){
            new_y += bar_velocity_;
        } else if (msg->point.y < 0.5) {
            new_y -= bar_velocity_;
        }
        current_y_ = new_y;
        
    }

    void game_state_callback(const custom_messages::msg::Gamestate::SharedPtr state){
        //read the value of the game state
        game_state_=state->state;

    }

    void timer_callback(){
        //publish the position of the bar
        auto message = geometry_msgs::msg::Point();                     

        message.x = current_x_;
        message.y = current_x_;  

        position_publisher_->publish(message);
    }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightBarPosition>());
  rclcpp::shutdown();
  return 0;
}
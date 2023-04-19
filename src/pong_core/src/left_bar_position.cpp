#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/windowsize.hpp"
#include "custom_messages/msg/gamestate.hpp"
#include "custom_messages/msg/barstate.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "custom_messages/srv/windowsize.hpp"

class LeftBarPosition : public rclcpp::Node
{
public:
    LeftBarPosition() : Node("left_bar_position"), count_(0)
    {
        // create subscriber for the position of the bar given by light
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("terminal_input", 10,
                                                                                    std::bind(&LeftBarPosition::position_callback, this, std::placeholders::_1));
        // create subscriber for the game state
        game_state_subscriber_ = this->create_subscription<custom_messages::msg::Gamestate>("game_state", 10,
                                                                                            std::bind(&LeftBarPosition::game_state_callback, this, std::placeholders::_1));

        // publisher for the left bar status (y position, width and height)
        position_publisher_ = this->create_publisher<custom_messages::msg::Barstate>("left_bar_state", 10);

        // this could also be in server i think
        bar_spacing_ = 0.02;
        bar_velocity_ = 5;

        // Set up a client to request window size from the server
        client_ = this->create_client<custom_messages::srv::Windowsize>("get_window_size");
        request_window_size();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LeftBarPosition::timer_callback, this));
    }

private:
    size_t count_;
    // for subscribing
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Gamestate>::SharedPtr game_state_subscriber_;
    // for publishing
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_messages::msg::Barstate>::SharedPtr position_publisher_;
    // client
    rclcpp::Client<custom_messages::srv::Windowsize>::SharedPtr client_;


    // Member variables to store current position and window size information
    float bar_spacing_;   // Spacing between the bar and the walls
    int bar_velocity_;    // set what is the speed of the bar
    int current_x_;       // Current y-coordinate of the bar
    int current_y_;       // Current y-coordinate of the bar
    int game_state_;      // for reading the state of the game
    int bar_half_height_; // for height if the bar
    int bar_half_width_;  // width of the bar
    int window_width_;    // Width of the window
    int window_height_;   // Height of the window

    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // new_y intialized as current on will be changed through this void
        int new_y = current_y_;
        RCLCPP_INFO(this->get_logger(), "Point '%f'", msg->y);

        // steering if the light is above half move up otherwise move down - the light point comes between 0 and 1
        // the more far away the light is the faster the bar moves
        if (msg->y > 0.8)
        {
            new_y += 2 * bar_velocity_;
        }
        else if (msg->y > 0.6 && msg->y <= 0.8)
        {
            new_y += bar_velocity_;
        }
        else if (msg->y > 0.2 && msg->y <= 0.4)
        {
            new_y -= bar_velocity_;
        }
        else if (msg->y <= 0.2)
        {
            new_y -= 2 * bar_velocity_;
        }
        // cases that the bar does not leave the frame
        //  full bar height = 2* bar_half_heigh_
        if (new_y + bar_half_height_ > window_height_)
        {
            new_y = window_height_ - bar_half_height_;
        }

        if (new_y - bar_half_height_ < 0)
        {
            new_y = bar_half_height_;
        }

        current_y_ = new_y;
        RCLCPP_INFO(this->get_logger(), "CALC'%d'", current_y_);
    }

    void game_state_callback(const custom_messages::msg::Gamestate::SharedPtr state)
    {
        // read the value of the game state
        game_state_ = state->state;
        if (game_state_ == 2 || game_state_ == 3)
        {
            current_y_ = window_height_ / 2;
            // Set the game state to in progress
            game_state_ = 1;
        }
        RCLCPP_INFO(this->get_logger(), "GameState'%d'", game_state_);
    }

    void timer_callback()
    {
        // publish the position of the bar, its width and height
        auto message = custom_messages::msg::Barstate();

        message.y_position = current_y_;
        message.x_position = current_x_;
        message.half_width = bar_half_height_;
        message.half_height = bar_half_width_;
        // also add x position

        position_publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Left_Height '%d'", this->window_height_);
        RCLCPP_INFO(this->get_logger(), "Left_Curr_y '%d'", current_y_);
    }

    void request_window_size()
    {

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service to be available.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        // Send a request to the service
        auto request = std::make_shared<custom_messages::srv::Windowsize::Request>();
        // request->a = 5; // Set the request value to 5
        auto result = send_request(request);
        if (result)
        {
            RCLCPP_INFO(this->get_logger(), "Received response: %d", result->width);
            // update the info based on the info from server
            window_height_ = result->height;
            window_width_ = result->width;
            // the bar dimensions and bar location initially depend on the window size
            current_y_ = window_height_ / 2;
            bar_half_height_ = window_height_ * 0.2;
            bar_half_width_ = window_width_ * 0.02;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
        }
        }

    std::shared_ptr<custom_messages::srv::Windowsize::Response> send_request(
        const std::shared_ptr<custom_messages::srv::Windowsize::Request> request)
    {
        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        else
        {
            return nullptr;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeftBarPosition>());
    rclcpp::shutdown();
    return 0;
}

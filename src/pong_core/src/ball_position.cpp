#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/gamestate.hpp"
#include "custom_messages/msg/ballstate.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "custom_messages/srv/windowsize.hpp"

class BallPosition : public rclcpp::Node
{
public:
    BallPosition() : Node("ball_position"), count_(0)
    {

        // create subscriber for the game state
        game_state_subscriber_ = this->create_subscription<custom_messages::msg::Gamestate>("game_state", 10,
                                                                                            std::bind(&BallPosition::game_state_callback, this, std::placeholders::_1));

        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("collision_velocity", 10, std::bind(&BallPosition::update_velocity, this, std::placeholders::_1));

        // Set up a client to request window size from the server
        client_ = this->create_client<custom_messages::srv::Windowsize>("get_window_size");
        request_window_size();

        // publisher for the left bar status (y position, width and height)
        position_publisher_ = this->create_publisher<custom_messages::msg::Ballstate>("ball_state", 10);

        // initial ball speed
        ball_speed_ = 15;
        // also make publisher for that
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&BallPosition::timer_callback, this));
    }

private:
    size_t count_;
    // for subscribing
    rclcpp::Subscription<custom_messages::msg::Gamestate>::SharedPtr game_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr velocity_subscriber_;

    // for publishing
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_messages::msg::Ballstate>::SharedPtr position_publisher_;
    // client
    rclcpp::Client<custom_messages::srv::Windowsize>::SharedPtr client_;


    // Member variables to store current position and window size information
    int current_x_;     // Current y-coordinate of the ball
    int current_y_;     // Current y-coordinate of the ball
    int game_state_;    // for reading the state of the game
    int window_width_;  // Width of the window
    int window_height_; // Height of the window
    float ball_speed_;
    float velocity_x_;
    float velocity_y_;
    int radius_;  // the size of the ball
    double angle; // use to get the velocity of the ball when restarted

    void game_state_callback(const custom_messages::msg::Gamestate::SharedPtr state)
    {
        // read the value of the game state
        game_state_ = state->state;
        RCLCPP_INFO(this->get_logger(), "GameState'%d'", game_state_);
        if (game_state_ == 2)
        {
            angle = M_PI / 4.0 + (double)rand() / RAND_MAX * M_PI / 2.0; // generates random angle between 45 and 135
            // set initial velocity values
            velocity_x_ = cos(angle);
            velocity_y_ = sin(angle);
            current_x_ = window_width_ / 2;
            current_y_ = window_height_ / 2;
            game_state_ = 1; // set to in progress
        }

        if (game_state_ == 3)
        {
            angle = M_PI / 4.0 + (double)rand() / RAND_MAX * M_PI / 2.0 + M_PI; // generates random angle between 225 and 315
            // set initial velocity values
            velocity_x_ = cos(angle);
            velocity_y_ = sin(angle);
            current_x_ = window_width_ / 2;
            current_y_ = window_height_ / 2;
            game_state_ = 1; // set to in progress
        }
    }

    // Update based on what is coming from the collision detector
    void update_velocity(const geometry_msgs::msg::Point::SharedPtr velocity)
    {
        // Update the velocity
        velocity_x_ = velocity->x;
        velocity_y_ = velocity->y;
    }

    void timer_callback()
    {

        // make sure the velocity is a normal vector
        // Compute velocity magnitude
        double velocity_magnitude = std::sqrt(velocity_x_ * velocity_x_ + velocity_y_ * velocity_y_);

        // update position
        current_x_ += ball_speed_ * velocity_x_ / velocity_magnitude;
        current_y_ += ball_speed_ * velocity_y_ / velocity_magnitude;

        // RCLCPP_INFO(this->get_logger(), "Ball position x: %d, y: %d", current_x_, current_y_);
        // RCLCPP_INFO(this->get_logger(), "Ball velocity x: %f, y: %f", velocity_x_, velocity_y_);
        // publish the position of the bar, its width and height
        auto message = custom_messages::msg::Ballstate();

        message.position.y = current_y_;
        message.position.x = current_x_;
        message.velocity.y = velocity_y_;
        message.velocity.x = velocity_x_;
        message.radius = radius_;

        position_publisher_->publish(message);
    }

    void request_window_size()
    {

        // Wait for the service to be available
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
            current_x_ = window_width_ / 2;
            radius_ = window_width_ * 0.005;
            // at the start the ball can go either left or right
            angle = M_PI / 4.0 + (double)rand() / RAND_MAX * M_PI / 2.0 + (int)rand() % 2; // generates random angle between 45 and 135 degress and 225 and 315
            // set initial velocity values
            velocity_x_ = cos(angle);
            velocity_y_ = sin(angle);
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
    rclcpp::spin(std::make_shared<BallPosition>());
    rclcpp::shutdown();
    return 0;
}

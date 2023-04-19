#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/ballstate.hpp"
#include "custom_messages/msg/barstate.hpp"
#include "custom_messages/msg/score.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "custom_messages/srv/windowsize.hpp"

// what will be used to handle collisions instead of having varibles
struct Vector2D
{
    double x;
    double y;
};

struct Ball
{
    Vector2D position;
    Vector2D velocity;
    double radius;
};

struct Bar
{
    Vector2D position;
    double half_width;
    double half_height;
};

class CollisionDetection : public rclcpp::Node
{
public:
    CollisionDetection() : Node("collision_detection"), count_(0)
    {
        // subscribers for bar states
        left_bar_state_subscriber_ = this->create_subscription<custom_messages::msg::Barstate>("left_bar_state", 10,
                                                                                               std::bind(&CollisionDetection::left_bar_state_callback, this, std::placeholders::_1));

        // right_bar_state_subscriber_ = this->create_subscription<custom_messages::msg::Barstate>("right_bar_state", 10,
        //                                                                                        std::bind(&CollisionDetection::right_bar_state_callback, this, std::placeholders::_1));
        // subscirber for ball position
        ball_state_subscriber_ = this->create_subscription<custom_messages::msg::Ballstate>("ball_state", 10,
                                                                                            std::bind(&CollisionDetection::ball_state_callback, this, std::placeholders::_1));
        // publisher for the left bar status (y position, width and height)
        ball_veloctity_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("collision_velocity", 10);
        // publisher for score
        score_publisher_ = this->create_publisher<custom_messages::msg::Score>("score", 10);

        // Set up a client to request window size from the server
        client_ = this->create_client<custom_messages::srv::Windowsize>("get_window_size");
        request_window_size();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CollisionDetection::timer_callback, this));
    }

private:
    size_t count_;
    // subscribers
    rclcpp::Subscription<custom_messages::msg::Barstate>::SharedPtr left_bar_state_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Barstate>::SharedPtr right_bar_state_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Ballstate>::SharedPtr ball_state_subscriber_;
    // publsiher for changed velocity
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_veloctity_publisher_;
    rclcpp::Publisher<custom_messages::msg::Score>::SharedPtr score_publisher_;

    // client
    rclcpp::Client<custom_messages::srv::Windowsize>::SharedPtr client_;

    rclcpp::TimerBase::SharedPtr timer_;

    // for the "top" and "bottom" of the screen collisions
    int window_width_;  // Width of the window
    int window_height_; // Height of the window
    int ball_pos_prev_; // not to count double scores


    // for the declared structs
    Ball ball_;
    Bar left_bar_;
    Bar right_bar_;

    void timer_callback()
    {
        collision_handler();
    }
    // update the positions
    void left_bar_state_callback(const custom_messages::msg::Barstate::SharedPtr left_bar)
    {
        left_bar_.position.x = left_bar->x_position;
        left_bar_.position.y = left_bar->y_position;
        left_bar_.half_width = left_bar->half_width;
        left_bar_.half_height = left_bar->half_height;
    }

    void right_bar_state_callback(const custom_messages::msg::Barstate::SharedPtr right_bar)
    {
        right_bar_.position.x = right_bar->x_position;
        right_bar_.position.y = right_bar->y_position;
        right_bar_.half_width = right_bar->half_width;
        right_bar_.half_height = right_bar->half_height;
    }

    void ball_state_callback(const custom_messages::msg::Ballstate::SharedPtr ball)
    {
        ball_.position.x = ball->position.x;
        ball_.position.y = ball->position.y;
        ball_.velocity.x = ball->velocity.x;
        ball_.velocity.y = ball->velocity.y;
        ball_.radius = ball->radius;
    }
    // boolean for checking if there was a collison with left bar
    bool ball_collides_with_left_bar(const Ball &ball, const Bar &bar)
    {
        // Check if the ball's position is within the boundaries of the bar
        if (ball.position.y > bar.position.y - bar.half_height - ball.radius &&
            ball.position.y < bar.position.y + bar.half_height + ball.radius &&
            ball.position.x >= bar.position.x - bar.half_width - ball.radius &&
            ball.position.x > bar.position.x + bar.half_width + ball.radius &&
            ball.velocity.x < 0 && ball.position.x < window_width_ / 2)
        {
            // Ball collided with the bar
            return true;
        }
        else
        {
            return false;
        }
    }

    // boolean for checking if there was a collison with right bar
    bool ball_collides_with_right_bar(const Ball &ball, const Bar &bar)
    {
        // Check if the ball's position is within the boundaries of the bar
        if (ball.position.y > bar.position.y - bar.half_height - ball.radius &&
            ball.position.y < bar.position.y + bar.half_height + ball.radius &&
            ball.position.x >= bar.position.x - bar.half_width - ball.radius &&
            ball.position.x > bar.position.x + bar.half_width + ball.radius &&
            ball.velocity.x > 0 && ball.position.x > window_width_ / 2)
        {
            // Ball collided with the bar
            return true;
        }
        else
        {
            return false;
        }
    }

    bool ball_collides_with_window(const Ball &ball)
    {
        // Check if the ball's touched the top or the bottom of the screen
        if (ball.position.y <= 0 + ball.radius || ball.position.y >= window_height_ - ball.radius)
        {
            // Ball collided with the bar
            return true;
        }
        else
        {
            return false;
        }
    }

    bool ball_exits_the_window_left(const Ball &ball)
    {
        // Check if the left the screen on the left
        if (ball.position.x <= 0 - ball.radius)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool ball_exits_the_window_right(const Ball &ball)
    {
        // Check if the left the screen on the right
        if (ball.position.x >= window_width_ + ball.radius)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void collision_handler()
    {
        // intialize the message for colliion

        if (ball_collides_with_left_bar(ball_, left_bar_) || ball_collides_with_right_bar(ball_, right_bar_))
        {
            auto velocity_message = geometry_msgs::msg::Point();

            ball_.velocity.x = -ball_.velocity.x;
            velocity_message.x = ball_.velocity.x;
            velocity_message.y = ball_.velocity.y;
            ball_veloctity_publisher_->publish(velocity_message);
        }

        if (ball_collides_with_window(ball_))
        {
            auto velocity_message = geometry_msgs::msg::Point();

            ball_.velocity.y = -ball_.velocity.y;
            velocity_message.x = ball_.velocity.x;
            velocity_message.y = ball_.velocity.y;
            ball_veloctity_publisher_->publish(velocity_message);
        }

        if (ball_exits_the_window_left(ball_))
        {
            auto score_message = custom_messages::msg::Score();
            // the right player scored
            score_message.score = 'R';
            RCLCPP_INFO(this->get_logger(), "Left right");

            score_publisher_->publish(score_message);
        }

        if (ball_exits_the_window_right(ball_))
        {
            auto score_message = custom_messages::msg::Score();
            // the left player scored
            score_message.score = 'L';
            RCLCPP_INFO(this->get_logger(), "Left scored");
            score_publisher_->publish(score_message);
        }
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
    rclcpp::spin(std::make_shared<CollisionDetection>());
    rclcpp::shutdown();
    return 0;
}

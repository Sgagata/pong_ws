#include "rclcpp/rclcpp.hpp"
#include "custom_messages/srv/windowsize.hpp"

class WindowSizeServer : public rclcpp::Node
{
public:
    WindowSizeServer()
        : Node("window_size_server")
    {
        // Declare the service
        service_ = this->create_service<custom_messages::srv::Windowsize>(
            "get_window_size", std::bind(&WindowSizeServer::handleWindowSize, this,
                                         std::placeholders::_1, std::placeholders::_2));
        // Initialize width and height
        width_ = 640;
        height_ = 480;
        RCLCPP_INFO(get_logger(), "init: %d, height: %d",
                    width_, height_);
    }

private:
    rclcpp::Service<custom_messages::srv::Windowsize>::SharedPtr service_;
    int32_t width_;
    int32_t height_;

    //the request has to be here otherwsie throws error
    void handleWindowSize(const std::shared_ptr<custom_messages::srv::Windowsize::Request> request,
                          const std::shared_ptr<custom_messages::srv::Windowsize::Response> response)
    {
        // Fill the response with the current width and height
        response->width = width_;
        response->height = height_;

        RCLCPP_INFO(get_logger(), "Received GetWindowSize request. Responding with width: %d, height: %d",
                    response->width, response->height);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WindowSizeServer>());
    rclcpp::shutdown();
    return 0;
}
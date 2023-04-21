//==============================================================
// Filename : pong_visualization.cppz
// Authors : Franka van Jaarsveld, Agata Sowa
// Group : 22
// License: N.A. or open source license like LGPL
// Description : Provides a visualization of the game
//==============================================================

#include <iostream>
#include <SDL2/SDL.h>

#include "../include/SDL2_UI.h"
#include "../include/Pong_field.h"

#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/ballstate.hpp"
#include "custom_messages/msg/barstate.hpp"
#include "custom_messages/msg/gamestate.hpp"
#include "custom_messages/msg/scorevalue.hpp"

/// @brief 
class PongVisualization : public rclcpp::Node
{
public:
    PongVisualization() : Node("pong_visualization")
    {
        // all the subscirbers for the visualization
        ball_subscriber_ = this->create_subscription<custom_messages::msg::Ballstate>("ball_state", 10,
                                                                                      std::bind(&PongVisualization::ball_callback, this, std::placeholders::_1));

        left_bar_subscriber_ = this->create_subscription<custom_messages::msg::Barstate>("left_bar_state", 10,
                                                                                         std::bind(&PongVisualization::left_bar_callback, this, std::placeholders::_1));

        right_bar_subscriber_ = this->create_subscription<custom_messages::msg::Barstate>("right_bar_state", 10,
                                                                                          std::bind(&PongVisualization::right_bar_callback, this, std::placeholders::_1));

        game_state_subscriber_ = this->create_subscription<custom_messages::msg::Gamestate>("game_state", 10,
                                                                                            std::bind(&PongVisualization::game_state_callback, this, std::placeholders::_1));

        score_subscriber_ = this->create_subscription<custom_messages::msg::Scorevalue>("score_value", 10,
                                                                                        std::bind(&PongVisualization::score_callback, this, std::placeholders::_1));
    }

    Pong_field field;

private:
    // all the subscibers for updating the game window
    rclcpp::Subscription<custom_messages::msg::Ballstate>::SharedPtr ball_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Barstate>::SharedPtr left_bar_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Barstate>::SharedPtr right_bar_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Gamestate>::SharedPtr game_state_subscriber_;
    rclcpp::Subscription<custom_messages::msg::Scorevalue>::SharedPtr score_subscriber_;

    void ball_callback(const custom_messages::msg::Ballstate::SharedPtr ball)
    {
        field.setBallRadius(ball->radius);
        field.setXYBall(ball->position.x, ball->position.y);
    }

    void left_bar_callback(const custom_messages::msg::Barstate::SharedPtr left_bar)
    {
        field.setBatWidth(left_bar->half_width);
        field.setLeftBatHeight(left_bar->half_height);
        field.setYBatLeft(left_bar->y_position);
        field.setXBatLeft(left_bar->x_position);
    }
    void right_bar_callback(const custom_messages::msg::Barstate::SharedPtr right_bar)
    {
        field.setBatWidth(right_bar->half_width);
        field.setRightBatHeight(right_bar->half_height);
        field.setYBatRight(right_bar->y_position);
        field.setXBatRight(right_bar->x_position);
    }
    void game_state_callback(const custom_messages::msg::Gamestate::SharedPtr game_state)
    {
        if (game_state->state == 0)
        {
            field.setFieldText(std::string("Game Over"));
        }
    }

    void score_callback(const custom_messages::msg::Scorevalue::SharedPtr score)
    {
        field.setFieldText(std::to_string(score->left_score) + std::string(" - ") + std::to_string(score->right_score));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PongVisualization>();

    bool quit = false;

    while (!quit)
    {
        // update the callbacks
        rclcpp::spin_some(node);

        // Show it on the screen
        node->field.DrawField();

        // We regularly need to let SDL process windows events such as mouse presses and keyboard presses
        // (even if they are not used; there are also windows events 'under water' that *are* used).
        // The function .processEvents() that does that also tells us if the user has pressed the quit button.
        bool wantsToQuit;
        wantsToQuit = node->field.sdl2_ui.processEvents();
        if (wantsToQuit)
        {
            quit = true;
        }

        // Wait for some time to simulate a loop rate
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();

    return 0;
} // end main
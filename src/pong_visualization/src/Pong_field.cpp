/* Pong_field.cpp */
#include <string>
#include "../include/Pong_field.h"

/** Set the vertical position of the left bat.
 * Units are pixles
 */
void Pong_field::setYBatLeft(double v_)
{
    yBatLeft = v_;
}

/** Set the vertical position of the right bat.
 * Units are pixels
 */
void Pong_field::setYBatRight(double v_)
{
    yBatRight = v_;
}

/** Set the vertical position of the left bat.
 * Units are pixles
 */
void Pong_field::setXBatLeft(double v_)
{
    xBatLeft = v_;
}

/** Set the vertical position of the right bat.
 * Units are pixels
 */
void Pong_field::setXBatRight(double v_)
{
    xBatRight = v_;
}

/** Set the horizontal and vertical position of the (center of) the ball.
 * Units pixels
 */
void Pong_field::setXYBall(double x_, double y_)
{
    xBall = x_;
    yBall = y_;
}

/** Set the radius of the bar
 * Units are pixels
 */
void Pong_field::setBallRadius(int radius_)
{
    ballSize = radius_;
}

/** Set the width of the bar
 * Units are pixels
 */
void Pong_field::setBatWidth(int half_width_)
{
    batWidth = half_width_;
}

/** Set the height of the left bar
 * Units are pixels
 */
void Pong_field::setLeftBatHeight(int half_height_)
{
    leftBatHeight = half_height_;
}

/** Set the height of the right bar
 * Units are pixels
 */
void Pong_field::setRightBatHeight(int half_height_)
{
    rightBatHeight = half_height_;
}

/** Set the text to display in the middle of the field.
 *  You can use this to show the score, "0-4"
 *  or any other text, "Game over".
 *  The text is shown until a new text is set.
 *  To remove the text, send an empty string, "".
 */
void Pong_field::setFieldText(std::string s_)
{
    fieldText = s_;
}

/** Draw the current state of all objects on the screen. Just call this
 * function regularly and you'll be fine...
 */
void Pong_field::DrawField()
{
    sdl2_ui.clear(); // Clear the draw buffer. This calls SDL2_UI::clear.

    // We draw each element in a different color. This helps with debugging.

    // Draw walls. They are static so we hard-code them here.
    sdl2_ui.setDrawColor(128, 128, 128, 255);                                                   // Walls are gray
    sdl2_ui.drawRectangle(0, 0, screenWidth - 1, wallHeight);                                   // Top wall
    sdl2_ui.drawRectangle(0, screenHeight - wallHeight - 1, screenWidth - 1, screenHeight - 1); // Bottom wall

    // Left bat
    sdl2_ui.setDrawColor(64, 64, 255, 255); // Left bat is bLue
    sdl2_ui.drawRectangle(xBatLeft - batWidth, yBatLeft - leftBatHeight, xBatLeft + batWidth, yBatLeft + leftBatHeight);

    // Right bat
    sdl2_ui.setDrawColor(255, 64, 64, 255); // Right bat is Red
    sdl2_ui.drawRectangle(xBatRight - batWidth, yBatRight - rightBatHeight, xBatRight + batWidth, yBatRight + rightBatHeight);

    // Ball
    sdl2_ui.setDrawColor(255, 255, 255, 255); // Ball is white
    sdl2_ui.drawRectangle(xBall - ballSize, yBall - ballSize, xBall + ballSize, yBall + ballSize);

    // Text
    sdl2_ui.drawText(fieldText, screenWidth / 2, screenHeight / 4);

    // Show it on the screen
    sdl2_ui.present(); // This calls SDL2_UI::present
}

/* Pong_field.h */
#include <string>
#include "SDL2_UI.h"

class Pong_field
{
private:
    // Size settings. All in pixels.
    static const int screenWidth = 1000;
    static const int screenHeight = 600;
    static const int fontSize = 160;

    static const int wallHeight = 30;
    // int batWidth;
    // int rightBatHeight;
    // int leftBatHeight;
    int batWidth = 30;
    int rightBatHeight = 100; // Make sure this is even
    int leftBatHeight = 100;  // Make sure this is even

    int ballSize = 30; // Make sure this is even

    // (copy of) state
    double yBatLeft = 0;  /// Vertical position of right bat. Units are  pixels
    double yBatRight = 0; /// Vertical position of right bat. Units are  pixels
    double xBatLeft = 0;  /// Vertical position of right bat. Units are  pixels
    double xBatRight = 0; /// Vertical position of right bat. Units are  pixels
    double xBall = 0;     /// Horizontal position of the (center of) the ball. Units are  pixels
    double yBall = 0;     /// Vertical position of the (center of) the ball. Units are  pixels
    std::string fieldText = "";

public:
    /** Constructor
     * Default
     */
    Pong_field() : sdl2_ui("Pong! - group 22", screenWidth, screenHeight, fontSize)
    {
    }

    /** Set the vertical position of the left bat.
     * Units are  pixels.
     */
    void setYBatLeft(double v);

    /** Set the vertical position of the right bat.
     * Units are  pixels.
     */
    void setYBatRight(double v);

    /** Set the horizontal position of the right bat.
     * Units are  pixels.
     */
    void setXBatRight(double v);

    /** Set the horizontal position of the left bat.
     * Units are  pixels.
     */
    void setXBatLeft(double v);

    /** Set the horizontal and vertical position of the (center of) the ball.
     * Units are  pixels.
     */
    void setXYBall(double x_, double y_);

    /** Set the radius of the ball
     * Units are  pixels.
     */
    void setBallRadius(int radius);

    /** Set the width of the bat
     * Units are  pixels.
     */
    void setBatWidth(int half_width_);

    /** Set the height of the left bat
     * Units are  pixels.
     */
    void setLeftBatHeight(int half_height_);

    /** Set the height of the left bat
     * Units are  pixels.
     */
    void setRightBatHeight(int half_height_);

    /** Set the text to display in the middle of the field.
     *  Use \n in the string for a new line. You can use this to show the score, "0-4"
     *  or any other text, "Game over".
     *  The text is shown until a new text is set.
     *  To remove the text, send an empty string, "".
     */
    void setFieldText(std::string s_);

    /** Draw the current state of all objects on the screen. Just call this function regularly and
     * you'll be fine...
     */
    void DrawField();

    /** The screen canvas. Public so that you can
     * interact with from outside the Pong_field class.
     */
    SDL2_UI sdl2_ui;
};
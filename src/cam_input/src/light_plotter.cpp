//==============================================================
// Filename : light_plotter.cpp
// Authors : Franka van Jaarsveld, Agata Sowa
// Group : 22
// License: N.A. or open source license like LGPL
// Description : Node supposed to determine the light oscilation
//==============================================================
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <complex>
#include <iostream>
#include <valarray>

typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;

class LightPlotter : public rclcpp::Node
{
public:
    LightPlotter() : Node("light_plotter")
    {

        // initialize x
        num_samples = 128;

        x.resize(num_samples);

        // initialize the position in the middle of the screen
        y_position = 0.5;
        y_position_prev = 0.5;
        idx = 0;
        counter = 0;
        // create a subsciber
        light_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("left_light_position", 10,
                                                                                 std::bind(&LightPlotter::light_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr light_subscriber_;
    CArray x;
    int idx;
    int counter;
    int num_samples;
    float y_position_prev;
    float y_position;

    void light_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // get the light position
        if (std::isnan(msg->y))
        {
            y_position = y_position_prev;
        }
        else
        {
            y_position = msg->y;
            y_position_prev = y_position;
        }
        RCLCPP_INFO(this->get_logger(), "Position: %f", y_position);

        // Add the new light position value to the input signal buffer
        x[idx] = y_position;
        //when there is more than num_samples it replaces values from the begig
        idx = (idx+1)% num_samples;

        RCLCPP_INFO(this->get_logger(), "Index %i", idx);

        // Perform FFT on the input signal buffer
        CArray fft_result = x;
        fft(fft_result);

        // Find the frequency with the highest magnitude
        double max_mag = 0.0;
        double max_freq = 0.0;
        for (size_t i = 0; i < num_samples / 2; ++i)
        {
            double mag = std::abs(fft_result[i]);
            if (mag > max_mag)
            {
                max_mag = mag;
                max_freq = static_cast<double>(i) / static_cast<double>(num_samples) * 2.0 * M_PI;
                RCLCPP_INFO(this->get_logger(), "Detected frequency: %f", fft_result[0]);
            }
        }
    }

    // implementation from https://rosettacode.org/wiki/Fast_Fourier_transform
    void fft(CArray &x)
    {
        const size_t N = x.size();
        if (N <= 1)
            return;

        // divide
        CArray even = x[std::slice(0, N / 2, 2)];
        CArray odd = x[std::slice(1, N / 2, 2)];

        // conquer
        fft(even);
        fft(odd);

        // combine
        for (size_t k = 0; k < N / 2; ++k)
        {
            Complex t = std::polar(1.0, -2 * M_PI * k / N) * odd[k];
            x[k] = even[k] + t;
            x[k + N / 2] = even[k] - t;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightPlotter>());

    rclcpp::shutdown();

    return 0;
}

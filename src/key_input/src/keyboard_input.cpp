// Code copied from turtlesim teleop_turtle_key.cpp,
// https://raw.githubusercontent.com/ros/ros_tutorials/foxy-devel/turtlesim/tutorials/teleop_turtle_key.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include <signal.h>
#include <stdio.h>
# include <termios.h>
# include <unistd.h>

class KeyboardReader
{
public:
  KeyboardReader()
    : kfd(0)
  {
    // put the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }

  void readOne(char * c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }

  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Define global variable input
KeyboardReader input;

class KeyboardInputNode
{
public:
  KeyboardInputNode();
  int keyLoop();
  rclcpp::Logger getLogger();
private:
  void spin();
  
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr keyCode_pub_;
};



KeyboardInputNode::KeyboardInputNode()
{
  nh_ = rclcpp::Node::make_shared("keyboard_input");
  keyCode_pub_ = nh_->create_publisher<std_msgs::msg::Int16>("/keyboard_input/key", 1);
}

rclcpp::Logger KeyboardInputNode::getLogger()
{
  return nh_->get_logger();
}

void KeyboardInputNode::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardInputNode::keyLoop()
{
  char c;

  std::thread{std::bind(&KeyboardInputNode::spin, this)}.detach();


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_INFO(nh_->get_logger(), "value: %3d = 0x%02X\n", c, c);
       
    std_msgs::msg::Int16 keyCode;
    keyCode.data = c;
    keyCode_pub_->publish(keyCode);
  }
  return 0;
}


void quit(int sig)
{
  (void)sig; // Suppress argument-not-used warning
  puts("Now stopping");
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

void printHelp(rclcpp::Logger l) {
  RCLCPP_INFO(l, "Reading from keyboard. Hit Ctrl-C to stop.");
  RCLCPP_INFO(l, " ");
  RCLCPP_INFO(l, "To make keyboard more responsive, try the following:");
  RCLCPP_INFO(l, "- Set the keyboard responiveness: ``xset r rate 50 50``");
  RCLCPP_INFO(l, " (note that this is system-wide. Revert afterwards with ``xset r rate 500 33``)");
  RCLCPP_INFO(l, "- Run this node from an xterm window (instead of a default Terminal window)");
  RCLCPP_INFO(l, "BE SURE to read the README.md file first though!");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardInputNode keyboard_input_node;
  rclcpp::Logger logger = keyboard_input_node.getLogger();
  printHelp(logger);

  signal(SIGINT,quit); // Bind ctrl-C to the (callback) function quit()

  int rc = keyboard_input_node.keyLoop();
  input.shutdown();
  rclcpp::shutdown();
  
  return rc;
}


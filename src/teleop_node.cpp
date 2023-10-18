#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include <chrono>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_SPACE 0x20
#define KEYCODE_Q 0x71

using namespace std::chrono_literals;

class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }

  void readOne(char *c)
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

KeyboardReader input;

class RobotTeleop
{
public:
  RobotTeleop() : dir_1(0),
                  dir_2(0),
                  pwm_1(0),
                  pwm_2(0),
                  desired_pwm(100)
  {
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use keys to move the robot.");
    puts("↑ : foward      ↓ : backward");
    puts("← : left        → : right");
    puts("space : stop");
  }

  void keyLoop()
  {
    char c;

    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    switch (c)
    {
    case KEYCODE_LEFT:
      dir_1 = 1;
      dir_2 = 1;
      pwm_1 = desired_pwm;
      pwm_2 = desired_pwm;
      break;
    case KEYCODE_RIGHT:
      dir_1 = 0;
      dir_2 = 0;
      pwm_1 = desired_pwm;
      pwm_2 = desired_pwm;
      break;
    case KEYCODE_UP:
      dir_1 = 0;
      dir_2 = 1;
      pwm_1 = desired_pwm;
      pwm_2 = desired_pwm;
      break;
    case KEYCODE_DOWN:
      dir_1 = 1;
      dir_2 = 0;
      pwm_1 = desired_pwm;
      pwm_2 = desired_pwm;
      break;
    case KEYCODE_SPACE:
      pwm_1 = 0;
      pwm_2 = 0;
      break;
    }

    return;
  }

  int dir_1, dir_2, pwm_1, pwm_2, desired_pwm;

private:
};

class RosCommunicator : public rclcpp::Node
{
public:
  RosCommunicator()
      : Node("tutorial_teleop")
  {
    timer_ = this->create_wall_timer(
        10ms, std::bind(&RosCommunicator::TimerCallback, this));
    robot_teleop = new RobotTeleop();
    publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/tutorial/teleop", 10);
  }

  void TimerCallback()
  {
    robot_teleop->keyLoop();
    auto message = std_msgs::msg::Int64MultiArray();
    message.data.push_back(robot_teleop->dir_1);
    message.data.push_back(robot_teleop->dir_2);
    message.data.push_back(robot_teleop->pwm_1);
    message.data.push_back(robot_teleop->pwm_2);
    publisher_->publish(message);
  }

  RobotTeleop *robot_teleop;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr publisher_;
};

void quit(int sig)
{
  (void)sig;
  rclcpp::shutdown();
  input.shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, quit);
  rclcpp::spin(std::make_shared<RosCommunicator>());
  quit(0);
  return 0;
}

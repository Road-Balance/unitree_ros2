#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_legged_msgs/msg/HighCmd.hpp"

enum HighCmdMode
{ 
  IDLE=0,
  FORCE_STAND,
  TARGET_VELOCITY,
  TARGET_POSITION,
  PATH_MODE
};

class MinimalSubscriber: public rclcpp::Node
{
private:
  void topic_callback(const std_msgs::msg::Joy::SharedPtr msg) const
  {
    std::cout << "1X : " << msg->axes[0] << std::endl;
    std::cout << "1Y : " << msg->axes[1] << std::endl;
    std::cout << "2X : " << msg->axes[3] << std::endl;
    std::cout << "2Y : " << msg->axes[4] << std::endl;

    if (msg->buttons[4] == 1)
      control_mode = HighCmdMode::TARGET_VELOCITY;
    else if (msg->buttons[5] == 1)
      control_mode = HighCmdMode::TARGET_POSITION;
    else if (msg->buttons[6] == 1)
      control_mode = HighCmdMode::PATH_MODE;
    else
      control_mode = HighCmdMode::IDLE;

    if (control_mode == 1 || control_mode == 0) {
      SendHighROS.mode = control_mode;

      // TODO: Check value orientation
      SendHighROS.euler[0] = msg->axes[0] * 0.3;
      SendHighROS.euler[1] = msg->axes[1] * 0.3;
      SendHighROS.euler[2] = msg->axes[3] * 0.3;
    } else if (control_mode == 2) {
      // basic trot mode
      SendHighROS.mode = control_mode;
      SendHighROS.gait_type = 1;

      SendHighROS.velocity[0] = msg->axes[0] * 0.3; // -1  ~ +1
      SendHighROS.velocity[1] = msg->axes[1] * 0.3; // -1  ~ +1
      SendHighROS.yaw_speed = msg->axes[2] * 0.3;
      SendHighROS.body_height = 0.1;
    } 

    publisher_->publish(SendHighROS);
    // TODO: SendHighROS << operator
    std::cout << std::endl;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  unitree_legged_msgs::msg::HighCmd SendHighROS;

  HighCmdMode control_mode;

public:
  MinimalSubscriber(): Node("minimal_subscriber")
  {
    publisher_ = this->create_publisher<unitree_legged_msgs::msg::HighCmd>("joy_highcmd", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
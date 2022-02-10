
#include <memory>
#include "unitree_legged_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

class CartesianPub : public rclcpp::Node {
private:
  rclcpp::Publisher<unitree_legged_msgs::msg::IMU>::SharedPtr m_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  unitree_legged_msgs::msg::IMU m_cart_msg;

  void timer_callback(){
      m_cart_msg.quaternion[0] = 1.0;
      m_cart_msg.quaternion[1] = 2.0;
      m_cart_msg.quaternion[2] = 3.0;
      m_pub->publish(m_cart_msg);
  }
  
public:
  CartesianPub() : Node("unitree_imu_pub_node") {
    RCLCPP_INFO(get_logger(), "Unitree IMU Pub Node Created");

    m_pub = create_publisher<unitree_legged_msgs::msg::IMU>("/imu", 10);
    m_timer = create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&CartesianPub::timer_callback, this));
  }

};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CartesianPub>());

  rclcpp::shutdown();

  return 0;
}
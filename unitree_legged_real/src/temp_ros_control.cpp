/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <chrono>
#include <memory>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "unitree_legged_msgs/msg/high_cmd.hpp"
#include "unitree_legged_msgs/msg/high_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"

// #include "unitree_legged_real/convert.h"

// using namespace UNITREE_LEGGED_SDK;

float roll       = 0.0;
float pitch      = 0.0;
float yaw        = 0.0;

float x_vel      = 0.0;
float y_vel      = 0.0;
float yaw_speed  = 0.0;

uint8_t mode     = 1;
uint8_t gaitType = 0;
float bodyHeight = 0;
float footRaiseHeight = 0;

auto dog_can_move = std_msgs::msg::Bool();
auto dogImu = sensor_msgs::msg::Imu();

void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	std::cout << msg->linear.x << std::endl;

	x_vel = msg->linear.x;
}

void mode_cb(const std_msgs::msg::UInt8::SharedPtr msg)
{
	mode = msg->data;
	std::cout << unsigned(mode) << std::endl;
}

void test()
{
  auto node = rclcpp::Node::make_shared("simple_node");

	auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, cmd_vel_cb);
	auto mode_sub = node->create_subscription<std_msgs::msg::UInt8>("dog_mode", 10, mode_cb);
	auto dog_imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 1000);
	auto dog_can_move_pub = node->create_publisher<std_msgs::msg::Bool>("dog_can_move", 1000);

	rclcpp::WallRate loop(500);

	while(rclcpp::ok())
	{
		dog_can_move.data = true;
		dog_can_move_pub->publish(dog_can_move);

		dogImu.orientation.x = 1.0;
		dog_imu_pub->publish(dogImu);

		loop.sleep();
		rclcpp::spin_some(node);
	}

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  test();

  rclcpp::shutdown();
  return 0;
}

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

auto dog_can_move = std_msgs::msg::Bool;
auto dogImu = sensor_msgs::msg::Imu;


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_node");

  RCLCPP_INFO(node->get_logger(), "Logger Test");

  rclcpp::shutdown();
  return 0;
}
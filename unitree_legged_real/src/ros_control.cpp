/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #include <ros/ros.h>
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

// #include "convert.h"
#include "unitree_legged_real/convert.h"

using namespace UNITREE_LEGGED_SDK;

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

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    // while(ros::ok){
    while(rclcpp::ok()){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("unitree_walk_node");
    
    // ros::Rate loop_rate(500);
    rclcpp::WallRate loop_rate(500);  // Hz

	auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, cmd_vel_cb);
	auto mode_sub = node->create_subscription<std_msgs::msg::UInt8>("dog_mode", 10, mode_cb);
	auto dog_imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 1000);
	auto dog_can_move_pub = node->create_publisher<std_msgs::msg::Bool>("dog_can_move", 1000);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::msg::HighCmd SendHighROS;
    unitree_legged_msgs::msg::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (rclcpp::ok()){

        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);

        SendHighROS.mode = 0;      
        SendHighROS.gait_type = 0;
        SendHighROS.speed_level = 0;
        SendHighROS.foot_raise_height = 0;
        SendHighROS.body_height = 0;
        SendHighROS.euler[0]  = 0;
        SendHighROS.euler[1] = 0;
        SendHighROS.euler[2] = 0;
        SendHighROS.velocity[0] = 0.0f;
        SendHighROS.velocity[1] = 0.0f;
        SendHighROS.yaw_speed = 0.0f;
        SendHighROS.reserve = 0;

        // std::cout << x_vel << std::endl;
        // std::cout << y_vel << std::endl;
        // std::cout << yaw_speed << std::endl;

        // std::cout << mode << std::endl;

        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);

        rclcpp::spin_some(node);

        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    
}
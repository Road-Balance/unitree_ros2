/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// #include <unitree_legged_msgs/HighCmd.h>
// #include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_msgs/msg/high_cmd.hpp"
#include "unitree_legged_msgs/msg/high_state.hpp"

// #include "convert.h"
#include "unitree_legged_real/convert.h"

using namespace UNITREE_LEGGED_SDK;

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

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::msg::HighCmd SendHighROS;
    unitree_legged_msgs::msg::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    // while (ros::ok()){
    while (rclcpp::ok()){
        motiontime = motiontime+2;
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

        if(motiontime > 0 && motiontime < 1000){
            SendHighROS.mode = 1;
            SendHighROS.euler[0] = -0.3;
        }
        if(motiontime > 1000 && motiontime < 2000){
            SendHighROS.mode = 1;
            SendHighROS.euler[0] = 0.3;
        }
        if(motiontime > 2000 && motiontime < 3000){
            SendHighROS.mode = 1;
            SendHighROS.euler[1] = -0.2;
        }
        if(motiontime > 3000 && motiontime < 4000){
            SendHighROS.mode = 1;
            SendHighROS.euler[1] = 0.2;
        }
        if(motiontime > 4000 && motiontime < 5000){
            SendHighROS.mode = 1;
            SendHighROS.euler[2] = -0.2;
        }
        if(motiontime > 5000 && motiontime < 6000){
            SendHighROS.mode = 1;
            SendHighROS.euler[2] = 0.2;
        }
        if(motiontime > 6000 && motiontime < 7000){
            SendHighROS.mode = 1;
            SendHighROS.body_height = -0.2;
        }
        if(motiontime > 7000 && motiontime < 8000){
            SendHighROS.mode = 1;
            SendHighROS.body_height = 0.1;
        }
        if(motiontime > 8000 && motiontime < 9000){
            SendHighROS.mode = 1;
            SendHighROS.body_height = 0.0;
        }
        if(motiontime > 9000 && motiontime < 11000){
            SendHighROS.mode = 5;
        }
        if(motiontime > 11000 && motiontime < 13000){
            SendHighROS.mode = 6;
        }
        if(motiontime > 13000 && motiontime < 14000){
            SendHighROS.mode = 0;
        }
        if(motiontime > 14000 && motiontime < 18000){
            SendHighROS.mode = 2;
            SendHighROS.gait_type = 2;
            SendHighROS.velocity[0] = 0.4f; // -1  ~ +1
            SendHighROS.yaw_speed = 2;
            SendHighROS.foot_raise_height = 0.1;
            // printf("walk\n");
        }
        if(motiontime > 18000 && motiontime < 20000){
            SendHighROS.mode = 0;
            SendHighROS.velocity[0] = 0;
        }
        if(motiontime > 20000 && motiontime < 24000){
            SendHighROS.mode = 2;
            SendHighROS.gait_type = 1;
            SendHighROS.velocity[0] = 0.2f; // -1  ~ +1
            SendHighROS.body_height = 0.1;
            // printf("walk\n");
        }
        if(motiontime>24000 ){
            SendHighROS.mode = 1;
        }

        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);

        // ros::spinOnce();
        rclcpp::spin_some(node);

        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    // ros::init(argc, argv, "walk_ros_mode");
    rclcpp::init(argc, argv);

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    
}
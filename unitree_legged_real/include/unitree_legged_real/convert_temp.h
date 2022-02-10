/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

// #include <unitree_legged_msgs/LowCmd.h>
// #include <unitree_legged_msgs/LowState.h>
// #include <unitree_legged_msgs/HighCmd.h>
// #include <unitree_legged_msgs/HighState.h>
// #include <unitree_legged_msgs/motor_cmd.h>
// #include <unitree_legged_msgs/MotorState.h>
// #include <unitree_legged_msgs/BmsCmd.h>
// #include <unitree_legged_msgs/BmsState.h>
// #include <unitree_legged_msgs/IMU.h>

#include "unitree_legged_msgs/msg/low_cmd.h"
#include "unitree_legged_msgs/msg/low_state.h"
#include "unitree_legged_msgs/msg/high_cmd.h"
#include "unitree_legged_msgs/msg/high_state.h"
#include "unitree_legged_msgs/msg/motor_cmd.h"
#include "unitree_legged_msgs/msg/motor_state.h"
#include "unitree_legged_msgs/msg/bms_cmd.h"
#include "unitree_legged_msgs/msg/bms_state.h"
#include "unitree_legged_msgs/msg/imu.h"

#include "unitree_legged_sdk/unitree_legged_sdk.h"

unitree_legged_msgs::msg::Cartesian ToRos(UNITREE_LEGGED_SDK::Cartesian& lcm){
    unitree_legged_msgs::msg::Cartesian ros;
    ros.x = lcm.x;
    ros.y = lcm.y;
    ros.z = lcm.z;
    return ros;
}

UNITREE_LEGGED_SDK::BmsCmd ToLcm(unitree_legged_msgs::msg::BmsCmd& ros){
    UNITREE_LEGGED_SDK::BmsCmd lcm;
    lcm.off = ros.off;
    for(int i(0); i<3; ++i){
        lcm.reserve[i] = ros.reserve[i];
    }
    return lcm;
}

unitree_legged_msgs::msg::BmsState ToRos(UNITREE_LEGGED_SDK::BmsState& lcm){
    unitree_legged_msgs::msg::BmsState ros;
    ros.version_h = lcm.version_h;
    ros.version_l = lcm.version_l;
    ros.bms_status = lcm.bms_status;
    ros.soc = lcm.soc;
    ros.current = lcm.current;
    ros.cycle = lcm.cycle;
    for(int i(0); i<2; ++i){
        ros.bq_ntc[i] = lcm.bq_ntc[i];
        ros.mcu_ntc[i] = lcm.mcu_ntc[i];
    }
    for(int i(0); i<10; ++i){
        ros.cell_vol[i] = lcm.cell_vol[i];
    }
    return ros;
}

unitree_legged_msgs::msg::IMU ToRos(UNITREE_LEGGED_SDK::IMU& lcm)
{
    unitree_legged_msgs::msg::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.accelerometer[0];
    ros.accelerometer[1] = lcm.accelerometer[1];
    ros.accelerometer[2] = lcm.accelerometer[2];
    ros.temperature = lcm.temperature;
    return ros;
}

unitree_legged_msgs::msg::MotorState ToRos(UNITREE_LEGGED_SDK::MotorState& lcm)
{
    unitree_legged_msgs::msg::MotorState ros;
    ros.mode = lcm.mode;
    ros.q = lcm.q;
    ros.dq = lcm.dq;
    ros.ddq = lcm.ddq;
    ros.tau_est = lcm.tau_est;
    ros.q_raw = lcm.q_raw;
    ros.dq_raw = lcm.dq_raw;
    ros.ddq_raw = lcm.ddq_raw;
    ros.temperature = lcm.temperature;
    ros.reserve[0] = lcm.reserve[0];
    ros.reserve[1] = lcm.reserve[1];
    return ros;
}

UNITREE_LEGGED_SDK::motor_cmd ToLcm(unitree_legged_msgs::msg::motor_cmd& ros, UNITREE_LEGGED_SDK::motor_cmd lcmType)
{
    UNITREE_LEGGED_SDK::motor_cmd lcm;
    lcm.mode = ros.mode;
    lcm.q = ros.q;
    lcm.dq = ros.dq;
    lcm.tau = ros.tau;
    lcm.Kp = ros.Kp;
    lcm.Kd = ros.Kd;
    lcm.reserve[0] = ros.reserve[0];
    lcm.reserve[1] = ros.reserve[1];
    lcm.reserve[2] = ros.reserve[2];
    return lcm;
}

unitree_legged_msgs::msg::LowState ToRos(UNITREE_LEGGED_SDK::LowState& lcm)
{
    unitree_legged_msgs::msg::LowState ros;
    ros.level_flag = lcm.level_flag;
    ros.comm_version = lcm.comm_version;
    ros.robot_id = lcm.robot_id;
    ros.sn = lcm.sn;
    ros.bandwidth = lcm.bandwidth;
    ros.imu = ToRos(lcm.imu);
    for(int i = 0; i<20; i++){
        ros.motor_state[i] = ToRos(lcm.motor_state[i]);
    }
    ros.bms = ToRos(lcm.bms);
    for(int i = 0; i<4; i++){
        ros.foot_force[i] = lcm.foot_force[i];
        ros.foot_force_est[i] = lcm.foot_force_est[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wireless_remote[i] = lcm.wireless_remote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::LowCmd ToLcm(unitree_legged_msgs::msg::LowCmd& ros, UNITREE_LEGGED_SDK::LowCmd lcmType)
{
    UNITREE_LEGGED_SDK::LowCmd lcm;
    lcm.level_flag = ros.level_flag;
    lcm.comm_version = ros.comm_version;
    lcm.robot_id = ros.robot_id;
    lcm.sn = ros.sn;
    lcm.bandwidth = ros.bandwidth;
    for(int i = 0; i<20; i++){
        lcm.motor_cmd[i] = ToLcm(ros.motor_cmd[i], lcm.motor_cmd[i]);
    }
    lcm.bms = ToLcm(ros.bms);
    for(int i = 0; i<40; i++){
        lcm.wireless_remote[i] = ros.wireless_remote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}

unitree_legged_msgs::msg::HighState ToRos(UNITREE_LEGGED_SDK::HighState& lcm)
{
    unitree_legged_msgs::msg::HighState ros;
    ros.level_flag = lcm.level_flag;
    ros.comm_version = lcm.comm_version;
    ros.robot_id = lcm.robot_id;
    ros.sn = lcm.sn;
    ros.bandwidth = lcm.bandwidth;
    ros.mode = lcm.mode;
    ros.progress = lcm.progress;
    ros.imu = ToRos(lcm.imu);
    ros.gait_type = lcm.gait_type;
    ros.foot_raise_height = lcm.foot_raise_height;
    ros.body_height = lcm.body_height;
    ros.yaw_speed = lcm.yaw_speed;
    ros.bms = ToRos(lcm.bms);
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;

    for(int i(0); i<3; ++i){
        ros.position[i] = lcm.position[i];
        ros.velocity[i] = lcm.velocity[i];
    }

    for(int i(0); i<4; ++i){
        ros.foot_position2body[i] = ToRos(lcm.foot_position2body[i]);
        ros.foot_speed2body[i] = ToRos(lcm.foot_speed2body[i]);
        ros.foot_force[i] = lcm.foot_force[i];
        ros.foot_force_est[i] = lcm.foot_force_est[i];
    }

    for(int i(0); i<20; ++i){
        ros.temperature[i] = lcm.temperature[i];
    }

    for(int i(0); i<40; ++i){
        ros.wireless_remote[i] = lcm.wireless_remote[i];
    }

    return ros;
}

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::msg::HighCmd& ros, UNITREE_LEGGED_SDK::HighCmd lcmType)
{
    UNITREE_LEGGED_SDK::HighCmd lcm;
    lcm.level_flag = ros.level_flag;
    lcm.comm_version = ros.comm_version;
    lcm.robot_id = ros.robot_id;
    lcm.sn = ros.sn;
    lcm.bandwidth = ros.bandwidth;
    lcm.mode = ros.mode;
    lcm.gait_type = ros.gait_type;
    lcm.speedLevel = ros.speedLevel;
    lcm.foot_raise_height = ros.foot_raise_height;
    lcm.body_height = ros.body_height;
    lcm.yaw_speed = ros.yaw_speed;
    lcm.bms = ToLcm(ros.bms);
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;

    for(int i(0); i<2; ++i){
        lcm.postion[i] = ros.postion[i];
        lcm.velocity[i] = ros.velocity[i];
    }

    for(int i(0); i<3; ++i){
        lcm.euler[i] = ros.euler[i];
    }

    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }

    for(int i = 0; i<40; i++){
        lcm.wireless_remote[i] = ros.wireless_remote[i];
    }

    return lcm;
}

#endif  // _CONVERT_H_
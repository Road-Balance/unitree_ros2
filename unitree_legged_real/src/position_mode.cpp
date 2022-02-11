#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// #include <unitree_legged_msgs/LowCmd.h>
// #include <unitree_legged_msgs/LowState.h>

#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"

// #include "convert.h"
#include "unitree_legged_real/convert.h"
#include "rclcpp/rclcpp.hpp"

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

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("simple_node");

    // ros::Rate loop_rate(500);
    rclcpp::WallRate loop_rate(500);  // Hz

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float kp[3] = {0};
    float kd[3] = {0};
    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::msg::LowCmd SendLowROS;
    unitree_legged_msgs::msg::LowState RecvLowROS;

    bool initiated_flag = false;  // initiate need time
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.level_flag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motor_cmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motor_cmd[i].q = PosStopF;        // 禁止位置环
        SendLowROS.motor_cmd[i].kp = 0;
        SendLowROS.motor_cmd[i].dq = VelStopF;        // 禁止速度环
        SendLowROS.motor_cmd[i].kd = 0;
        SendLowROS.motor_cmd[i].tau = 0;
    }

    // while (ros::ok()){
    while (rclcpp::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FR_2 position: %f\n",  RecvLowROS.motor_state[FR_2].q);

        if(initiated_flag == true){
            motiontime++;

            SendLowROS.motor_cmd[FR_0].tau = -0.65f;
            SendLowROS.motor_cmd[FL_0].tau = +0.65f;
            SendLowROS.motor_cmd[RR_0].tau = -0.65f;
            SendLowROS.motor_cmd[RL_0].tau = +0.65f;

            // printf("%d\n", motiontime);
            // printf("%d %f %f %f\n", FR_0, RecvLowROS.motor_state[FR_0].q, RecvLowROS.motor_state[FR_1].q, RecvLowROS.motor_state[FR_2].q);
            // printf("%f %f \n",  RecvLowROS.motor_state[FR_0].mode, RecvLowROS.motor_state[FR_1].mode);
            if( motiontime >= 0){
                // first, get record initial position
                // if( motiontime >= 100 && motiontime < 500){
                if( motiontime >= 0 && motiontime < 10){
                    qInit[0] = RecvLowROS.motor_state[FR_0].q;
                    qInit[1] = RecvLowROS.motor_state[FR_1].q;
                    qInit[2] = RecvLowROS.motor_state[FR_2].q;
                }
                if( motiontime >= 10 && motiontime < 400){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/200.0;                       // needs count to 200
                    kp[0] = 5.0; kp[1] = 5.0; kp[2] = 5.0;
                    kd[0] = 1.0; kd[1] = 1.0; kd[2] = 1.0;
                    
                    qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                    qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                    qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
                }
                double sin_joint1, sin_joint2;
                // last, do sine wave
                if( motiontime >= 400){
                    sin_count++;
                    sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
                    sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
                    qDes[0] = sin_mid_q[0];
                    qDes[1] = sin_mid_q[1];
                    qDes[2] = sin_mid_q[2] + sin_joint2;
                    // qDes[2] = sin_mid_q[2];
                }

                SendLowROS.motor_cmd[FR_0].q = qDes[0];
                SendLowROS.motor_cmd[FR_0].dq = 0;
                SendLowROS.motor_cmd[FR_0].kp = kp[0];
                SendLowROS.motor_cmd[FR_0].kd = kd[0];
                SendLowROS.motor_cmd[FR_0].tau = -0.65f;

                SendLowROS.motor_cmd[FR_1].q = qDes[1];
                SendLowROS.motor_cmd[FR_1].dq = 0;
                SendLowROS.motor_cmd[FR_1].kp = kp[1];
                SendLowROS.motor_cmd[FR_1].kd = kd[1];
                SendLowROS.motor_cmd[FR_1].tau = 0.0f;

                SendLowROS.motor_cmd[FR_2].q =  qDes[2];
                SendLowROS.motor_cmd[FR_2].dq = 0;
                SendLowROS.motor_cmd[FR_2].kp = kp[2];
                SendLowROS.motor_cmd[FR_2].kd = kd[2];
                SendLowROS.motor_cmd[FR_2].tau = 0.0f;
            }
        }

        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);
        
        // ros::spinOnce();
        rclcpp::spin_some(node);
        
        loop_rate.sleep();

        count++;
        if(count > 10){
            count = 10;
            initiated_flag = true;
        }
    }
    return 0;
}

int main(int argc, char *argv[]){

    // ros::init(argc, argv);
    rclcpp::init(argc, argv);

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}
uint8 level_flag
uint16 comm_version                 # Old version Aliengo does not have
uint16 robot_id                     # Old version Aliengo does not have
uint32 sn                           # Old version Aliengo does not have
uint8 bandwidth                     # Old version Aliengo does not have
uint8 mode
float32 progress                    # new on Go1, reserve
IMU imu
uint8 gait_type                     # new on Go1, 0.idle  1.trot  2.trot running  3.climb stair
float32 foot_raise_height           # (unit: m, default: 0.08m), foot up height while walking
float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32 body_height                 # (unit: m, default: 0.28m)
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 yaw_speed                   # (unit: rad/s), rotateSpeed in body frame        
Cartesian[4] foot_position2body     # foot position relative to body
Cartesian[4] foot_speed2body        # foot speed relative to body
int8[20] temperature
BmsState bms
int16[4] foot_force                 # Old version Aliengo is different
int16[4] foot_force_est             # Old version Aliengo does not have
uint8[40] wireless_remote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc
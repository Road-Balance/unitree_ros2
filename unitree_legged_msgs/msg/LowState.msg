uint8 level_flag
uint16 comm_version             # Old version Aliengo does not have
uint16 robot_id                 # Old version Aliengo does not have
uint32 sn                       # Old version Aliengo does not have
uint8 bandwidth                 # Old version Aliengo does not have
IMU imu
MotorState[20] motor_state
BmsState bms                    # new on Go1, battery state
int16[4] foot_force             # force sensors     # Old version Aliengo is different
int16[4] foot_force_est         # force sensors     # Old version Aliengo does not have
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wireless_remote       # wireless commands
uint32 reserve                  # Old version Aliengo does not have
uint32 crc

# Old version Aliengo does not have:
Cartesian[4] ee_force_raw
Cartesian[4] ee_force         #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
Cartesian position            # will delete
Cartesian velocity            # will delete
Cartesian velocity_w          # will delete

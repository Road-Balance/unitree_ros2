uint8 level_flag                 
uint16 comm_version             # Old version Aliengo does not have
uint16 robot_id                 # Old version Aliengo does not have
uint32 sn                       # Old version Aliengo does not have
uint8 bandwidth                 # Old version Aliengo does not have
MotorCmd[20] motor_cmd
BmsCmd bms                      # new on Go1, battery command
# LED[4] led
uint8[40] wireless_remote
uint32 reserve                  # Old version Aliengo does not have
uint32 crc

Cartesian[4] ff               # will delete # Old version Aliengo does not have
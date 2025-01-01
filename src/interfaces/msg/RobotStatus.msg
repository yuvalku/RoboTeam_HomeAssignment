# RobotStatus.msg

std_msgs/Header header

int16 left_rpm
int16 left_load
int16 right_rpm
int16 right_load

int16 left_current
int16 right_current

uint32 left_odom
uint32 right_odom

int16 roll_deg
int16 pitch_deg
int16 yaw_deg

uint16 battery_current
uint16 battery_voltage
uint8 battery_charge

# For Probot extra
int16 left_temp
uint8 left_heat
int16 right_temp
uint8 right_heat

# BIT fields
uint8 left_bit_error
uint8 right_bit_error
uint8 battery_bit_error
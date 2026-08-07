std_msgs/Header header
float64 target_steering_angle # radians
float64 target_speed # m/s
float64 target_acceleration # m/s^2
uint8 gear_command # 0: NONE, 1: DRIVE, 2: NEUTRAL, 3: REVERSE, 4: PARK
bool emergency_stop

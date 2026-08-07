std_msgs/Header header
uint8 STATE_INIT=0
uint8 STATE_MANUAL=1
uint8 STATE_AUTONOMOUS=2
uint8 STATE_FAULT=3

uint8 current_state
bool is_healthy
string status_message

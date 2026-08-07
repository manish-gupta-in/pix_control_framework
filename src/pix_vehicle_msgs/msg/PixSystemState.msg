std_msgs/Header header

# System state constants
uint8 STATE_MANUAL     = 0   # Human control, DBW disabled
uint8 STATE_STANDBY    = 1   # DBW ready, no active algorithm
uint8 STATE_AUTONOMOUS = 2   # Algorithm actively controlling
uint8 STATE_FAULT      = 3   # Fault detected, safe fallback active
uint8 STATE_ESTOP      = 4   # Emergency stop latched

uint8 state                   # Current system state (see constants above)
string reason                 # Human-readable reason for state
float64 state_duration_secs   # Time in current state (seconds)
bool estop_latched            # True if E-stop is latched (requires manual clear)
bool fault_latched            # True if a recoverable fault is active
string active_algorithm       # Name of currently active algorithm (if AUTONOMOUS)
uint8 fault_count             # Total fault events since boot

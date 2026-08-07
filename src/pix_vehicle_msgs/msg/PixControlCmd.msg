std_msgs/Header header

# Steering control
float32 steer_target        # Target steering angle (deg), range: [-500, 500] (positive: right, negative: left)
float32 steer_speed         # Steering speed (deg/s), range: [1, 250]
bool steer_en               # Enable steering control

# Longitudinal control (Drive & Brake)
float32 speed_target        # Target vehicle speed (m/s)
float32 accel_target        # Target acceleration (m/s^2), range: [0, 10]
bool drive_en               # Enable speed/acceleration control

float32 brake_target        # Target brake pedal (%), range: [0, 100]
bool brake_en               # Enable brake control

# Gear & Park control constants
uint8 GEAR_TARGET_INVALID = 0
uint8 GEAR_TARGET_PARK = 1
uint8 GEAR_TARGET_REVERSE = 2
uint8 GEAR_TARGET_NEUTRAL = 3
uint8 GEAR_TARGET_DRIVE = 4

uint8 PARK_TARGET_RELEASE = 0
uint8 PARK_TARGET_PARKING_TRIGGER = 1

uint8 gear_target           # Target gear (see constants above)
bool gear_en                # Enable gear control
uint8 park_target           # Target park (see constants above)
bool park_en                # Enable park control


# Auxiliary control
uint8 turn_light_ctrl       # Turn lights: 0: OFF, 1: LEFT, 2: RIGHT, 3: HAZARD
bool headlight_ctrl         # Headlight control

# Safety
bool emergency_stop         # E-stop flag (forces zero command & brake enable)

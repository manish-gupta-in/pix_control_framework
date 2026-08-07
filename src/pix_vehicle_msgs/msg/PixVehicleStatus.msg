std_msgs/Header header

# Steering Feedback
float32 steer_angle         # Current steering angle (deg), range: [-500, 500]
float32 steer_speed         # Current steering speed (deg/s)
uint8 steer_en_state        # Steering control mode status: 0: Manual, 1: Auto, 2: Takeover, 3: Standby

# Longitudinal Feedback
float32 vehicle_speed       # Current vehicle speed (m/s)
float32 vehicle_accel       # Current vehicle acceleration (m/s^2)
float32 throttle_pedal      # Current throttle pedal position (%)
float32 brake_pedal         # Current brake pedal position (%)
uint8 drive_en_state        # Drive control mode status: 0: Manual, 1: Auto, 2: Takeover, 3: Standby
uint8 brake_en_state        # Brake control mode status: 0: Manual, 1: Auto, 2: Takeover, 3: Standby

# Gear & Park Feedback constants
uint8 GEAR_ACTUAL_INVALID = 0
uint8 GEAR_ACTUAL_PARK = 1
uint8 GEAR_ACTUAL_REVERSE = 2
uint8 GEAR_ACTUAL_NEUTRAL = 3
uint8 GEAR_ACTUAL_DRIVE = 4

uint8 PARK_ACTUAL_RELEASE = 0
uint8 PARK_ACTUAL_PARKING_TRIGGER = 1

uint8 gear_actual           # Current gear (see constants above)
uint8 park_actual           # Current park status (see constants above)


# General status
uint8 vehicle_mode          # Vehicle Mode state: 0: Manual Remote, 1: Auto, 2: Emergency, 3: Standby
uint8 drive_mode_status     # Drive Mode status: 0: Throttle Pedal, 1: Speed
uint8 steer_mode_status     # Steer Mode status: 0: Standard, 1: Non Direction, 2: Sync Direction
bool front_crash            # Front crash sensor state
bool back_crash             # Back crash sensor state
bool aeb_active             # AEB active state
bool brake_light_actual     # Brake light state
uint8 turn_light_actual     # Turn lights state: 0: OFF, 1: LEFT, 2: RIGHT, 3: HAZARD

# Fault status
uint8 steer_flt1
uint8 steer_flt2
uint8 drive_flt1
uint8 drive_flt2
uint8 brake_flt1
uint8 brake_flt2
uint8 park_flt
uint8 gear_flt

# Battery report
float32 battery_voltage     # Battery voltage (V)
float32 battery_current     # Battery current (A)
float32 battery_soc         # Battery state of charge (%)

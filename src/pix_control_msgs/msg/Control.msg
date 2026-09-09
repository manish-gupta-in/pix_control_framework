# PIX Framework Control Command
# Independent message type for the PIX Control Framework.
# Used on /pix_framework/control_cmd topic.

builtin_interfaces/Time stamp

# Lateral control
float64 steering_tire_angle          # [rad] target tire angle
float64 steering_tire_rotation_rate  # [rad/s] target tire rotation rate

# Longitudinal control
float64 velocity                     # [m/s] target velocity
float64 acceleration                 # [m/s^2] target acceleration

#!/usr/bin/env python3
import rclpy
from pix_vehicle_msgs.msg import PixControlCmd
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class MPCPlannerNode(BaseAlgorithmInterface):
    """
    Model Predictive Control (MPC) Planner.
    Solves for the optimal trajectory (steer, speed) over a prediction horizon.
    """
    def __init__(self):
        super().__init__('mpc_planner', '/pix/commands/cruise_control')
        self.timer = self.create_timer(0.05, self.plan_trajectory)
        
    def plan_trajectory(self):
        status = self.get_vehicle_status()
        
        # MOCK MPC LOGIC:
        # Here you would use CasADi or OSQP to solve:
        # min sum( Q*(x_k - x_ref)^2 + R*(u_k)^2 )
        # subject to: vehicle dynamics, obstacle constraints
        
        # For demonstration, we just command a smooth curve.
        optimal_steer = 15.0 # degrees
        optimal_speed = 2.5  # m/s
        
        self.publish_control_cmd(
            drive_en=True, speed_target=optimal_speed, accel_target=1.0,
            steer_en=True, steer_target=optimal_steer, steer_speed=100.0,
            brake_en=False, brake_target=0.0, gear_en=True, gear_target=4
        )

def main(args=None):
    rclpy.init(args=args)
    node = MPCPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

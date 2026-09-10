#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd
import time

class ControlArbitratorNode(Node):
    """
    Control Arbitrator (Multiplexer).
    Takes commands from AEB, Joystick, MPC, and GNSS nodes.
    Selects the highest priority command and sends it to the Safety Manager.
    
    Priority:
    1. AEB (Autonomous Emergency Braking)
    2. Joystick (Manual Override)
    3. MPC / GNSS (Autonomous Planners)
    """
    def __init__(self):
        super().__init__('control_arbitrator')
        
        # Subscriptions to all planners
        self.aeb_sub = self.create_subscription(PixControlCmd, '/pix_autonomy/aeb_cmd', self.aeb_cb, 10)
        self.joy_sub = self.create_subscription(PixControlCmd, '/pix_autonomy/joy_cmd', self.joy_cb, 10)
        self.mpc_sub = self.create_subscription(PixControlCmd, '/pix_autonomy/mpc_cmd', self.mpc_cb, 10)
        self.gnss_sub = self.create_subscription(PixControlCmd, '/pix_autonomy/gnss_cmd', self.gnss_cb, 10)
        self.lateral_sub = self.create_subscription(PixControlCmd, '/pix_autonomy/lateral_cmd', self.lateral_cb, 10)
        self.straight_sub = self.create_subscription(PixControlCmd, '/pix_autonomy/straight_cmd', self.straight_cb, 10)
        
        # Publisher to Safety Manager
        self.cmd_pub = self.create_publisher(PixControlCmd, '/pix/raw_control_cmd', 10)
        
        self.latest_cmds = {
            'aeb': (None, 0.0),
            'joy': (None, 0.0),
            'mpc': (None, 0.0),
            'gnss': (None, 0.0),
            'lateral': (None, 0.0),
            'straight': (None, 0.0),
        }
        
        self.timer = self.create_timer(0.02, self.arbitrate_loop) # 50Hz output
        
    def _update_cmd(self, source, msg):
        self.latest_cmds[source] = (msg, time.time())
        
    def aeb_cb(self, msg): self._update_cmd('aeb', msg)
    def joy_cb(self, msg): self._update_cmd('joy', msg)
    def mpc_cb(self, msg): self._update_cmd('mpc', msg)
    def gnss_cb(self, msg): self._update_cmd('gnss', msg)
    def lateral_cb(self, msg): self._update_cmd('lateral', msg)
    def straight_cb(self, msg): self._update_cmd('straight', msg)
        
    def arbitrate_loop(self):
        now = time.time()
        active_cmd = None
        source_name = "None"
        
        # Check AEB First (Timeout 0.5s)
        if self.latest_cmds['aeb'][0] and (now - self.latest_cmds['aeb'][1]) < 0.5:
            active_cmd = self.latest_cmds['aeb'][0]
            source_name = "AEB"
            
        # Then Joystick (Timeout 0.5s)
        elif self.latest_cmds['joy'][0] and (now - self.latest_cmds['joy'][1]) < 0.5:
            active_cmd = self.latest_cmds['joy'][0]
            source_name = "Joystick"
            
        # Then Lateral Avoidance (Timeout 0.5s)
        elif self.latest_cmds['lateral'][0] and (now - self.latest_cmds['lateral'][1]) < 0.5:
            active_cmd = self.latest_cmds['lateral'][0]
            source_name = "Lateral Avoidance"
            
        # Then MPC or GNSS (Timeout 1.0s)
        elif self.latest_cmds['mpc'][0] and (now - self.latest_cmds['mpc'][1]) < 1.0:
            active_cmd = self.latest_cmds['mpc'][0]
            source_name = "MPC"
            
        elif self.latest_cmds['gnss'][0] and (now - self.latest_cmds['gnss'][1]) < 1.0:
            active_cmd = self.latest_cmds['gnss'][0]
            source_name = "GNSS"
            
        elif self.latest_cmds['straight'][0] and (now - self.latest_cmds['straight'][1]) < 1.0:
            active_cmd = self.latest_cmds['straight'][0]
            source_name = "Straight Drive"
            
        # Output chosen command
        if active_cmd:
            self.cmd_pub.publish(active_cmd)
        else:
            # Idle / Standby
            idle = PixControlCmd()
            self.cmd_pub.publish(idle)

def main(args=None):
    rclpy.init(args=args)
    node = ControlArbitratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

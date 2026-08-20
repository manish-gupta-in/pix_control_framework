#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pix_vehicle_msgs.msg import PixControlCmd
from pix_algorithm_api import BaseAlgorithmInterface

import os
import cv2
import numpy as np
import time
from collections import deque

try:
    from ultralytics import YOLO
except ImportError:
    # We will log error if ultralytics is missing
    YOLO = None

class Ramp:
    def __init__(self, initial=0.0, rate=200.0, dt=0.02):
        self._pos = float(initial)
        self._target = float(initial)
        self._step = rate * dt

    def set_target(self, target, max_val):
        self._target = max(-max_val, min(max_val, float(target)))

    def step(self):
        diff = self._target - self._pos
        if abs(diff) <= self._step:
            self._pos = self._target
        else:
            self._pos += self._step if diff > 0 else -self._step
        return self._pos

    @property
    def pos(self):
        return self._pos

    @property
    def at_target(self):
        return abs(self._pos - self._target) < 0.5


class LateralAvoidanceController:
    def __init__(self, gain, max_avoidance, deadband, hold_frames, ramp_rate, dt=0.02):
        self.gain = float(gain)
        self.max_avoidance = float(max_avoidance)
        self.deadband = float(deadband)
        self.hold_frames = int(hold_frames)
        self.ramp = Ramp(0.0, ramp_rate, dt)
        self._hold_ctr = 0      # countdown after last detection
        self._last_offset = 0.0
        self._latch_side = 0    # -1=person came from LEFT, +1=RIGHT, 0=unknown
        self._flip_ctr = 0      # confirmation counter before flipping latch

    def update(self, detections, frame_w, frame_h):
        MIN_STEER_FRAC = 0.10   # minimum steer even when person at edge (10% of max)
        LATCH_FLIP_BAND = 0.25  # person must cross this far to the other side to flip
        LATCH_CONFIRM_FRAMES = 6 # must stay on new side this many frames before flip

        min_steer = MIN_STEER_FRAC * self.max_avoidance

        # ── 1. Pick best detection (closest / largest bounding box) ──────────────────
        if detections:
            best = max(detections, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
            x1, y1, x2, y2, conf = best
            cx = (x1 + x2) / 2.0
            norm_offset = max(-1.0, min(1.0, (cx - frame_w / 2.0) / (frame_w / 2.0)))
            self._last_offset = norm_offset
            self._hold_ctr = self.hold_frames
            state = "AVOID"

            # ── Latch side logic ──
            if self._latch_side == 0:
                self._latch_side = -1 if norm_offset <= 0 else +1
                self._flip_ctr = 0

            opposite_side = +1 if self._latch_side == -1 else -1
            if norm_offset * opposite_side > LATCH_FLIP_BAND:
                self._flip_ctr += 1
                if self._flip_ctr >= LATCH_CONFIRM_FRAMES:
                    self._latch_side = opposite_side
                    self._flip_ctr = 0
            else:
                self._flip_ctr = max(0, self._flip_ctr - 1)
        else:
            # ── No detection ──
            if self._hold_ctr > 0:
                self._hold_ctr -= 1
                norm_offset = self._last_offset
                state = "HOLD"
            else:
                self._latch_side = 0
                self._flip_ctr = 0
                self.ramp.set_target(0.0, self.max_avoidance)
                pos = self.ramp.step()
                return pos, {
                    "state": "CENTER",
                    "offset": 0.0,
                    "target": 0.0,
                    "ramp_pos": round(float(pos), 1),
                }

        # ── 2. Proximity magnitude ──
        proximity = 1.0 - abs(norm_offset)  # 0..1 (1 near center, 0 near edge)
        magnitude = min_steer + proximity * (self.max_avoidance - min_steer)
        magnitude = float(np.clip(magnitude, min_steer, self.max_avoidance))

        # ── 3. Direction from latched side ──
        # If person is on left (latch_side <= 0), we steer LEFT (-) physically
        # wait! Let's check original logic comments:
        # "Person on LEFT -> steer RIGHT (+)
        #  Person on RIGHT -> steer LEFT (-)"
        # And in update:
        # "target = -magnitude if self._latch_side <= 0 else +magnitude"
        # Wait, if person is on LEFT, _latch_side <= 0, so target = -magnitude (steer LEFT?)
        # Let's check the original code!
        # "raw = (-angle + 500)" -> VCU inverts steering.
        # "We pass the avoidance direction WITHOUT an extra negation so the net result is correct."
        # Ah! In the original code, target = -magnitude if self._latch_side <= 0 else +magnitude
        # Let's match the original code EXACTLY to maintain vehicle hardware sign convention compatibility!
        target = -magnitude if self._latch_side <= 0 else +magnitude
        target = float(np.clip(target, -self.max_avoidance, self.max_avoidance))

        # ── 4. Ramp ──
        self.ramp.set_target(target, self.max_avoidance)
        pos = self.ramp.step()

        return pos, {
            "state": state,
            "offset": round(float(norm_offset), 3),
            "target": round(target, 1),
            "ramp_pos": round(float(pos), 1),
            "side": "L" if self._latch_side == -1 else ("R" if self._latch_side == 1 else "?"),
        }


class YoloPersonAvoidanceNode(BaseAlgorithmInterface):
    def __init__(self):
        super().__init__('yolo_person_avoidance', '/pix/commands/human_avoidance')
        
        # Declare parameters
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('gain', 300.0)
        self.declare_parameter('max_avoidance', 500.0)
        self.declare_parameter('deadband', 0.08)
        self.declare_parameter('ramp_rate', 200.0)
        self.declare_parameter('hold_frames', 15)
        self.declare_parameter('speed_dps', 250.0)
        self.declare_parameter('target_speed', 2.0)  # m/s target speed
        self.declare_parameter('no_display', True)    # Default headless
        self.declare_parameter('steer_only_mode', False)  # Stationary steer test
        
        self.yolo_model = self.get_parameter('yolo_model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.gain = self.get_parameter('gain').value
        self.max_avoidance = self.get_parameter('max_avoidance').value
        self.deadband = self.get_parameter('deadband').value
        self.ramp_rate = self.get_parameter('ramp_rate').value
        self.hold_frames = self.get_parameter('hold_frames').value
        self.speed_dps = self.get_parameter('speed_dps').value
        self.target_speed = self.get_parameter('target_speed').value
        self.no_display = self.get_parameter('no_display').value
        self.steer_only_mode = self.get_parameter('steer_only_mode').value
        
        if self.steer_only_mode:
            self.get_logger().warn(
                "STEER_ONLY_MODE=True: Vehicle will NOT move. "
                "Only steering actuation is active. Safe for stationary tests."
            )
        
        # YOLO Initialization
        if YOLO is None:
            self.get_logger().error("Ultralytics library not available! Please pip install ultralytics.")
            self.model = None
        else:
            self.get_logger().info(f"Loading YOLO model: {self.yolo_model} ...")
            self.model = YOLO(self.yolo_model)
            self.get_logger().info("YOLO Model loaded.")
            
        self.bridge = CvBridge()
        
        # Avoidance Controller (running at 50Hz equivalent frame processing)
        self.ctrl = LateralAvoidanceController(
            gain=self.gain,
            max_avoidance=self.max_avoidance,
            deadband=self.deadband,
            hold_frames=self.hold_frames,
            ramp_rate=self.ramp_rate,
            dt=0.02
        )
        
        # Camera subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/right/image',
            self.image_callback,
            10
        )
        
        # FPS Tracker
        self.fps_deque = deque(maxlen=30)
        self.last_frame_time = time.time()
        
        # Open display window if enabled
        if not self.no_display:
            cv2.namedWindow("YOLO Person Avoidance", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("YOLO Person Avoidance", 960, 540)
            
    def image_callback(self, msg):
        if self.model is None:
            return
            
        t0 = time.time()
        try:
            # Convert ROS2 image message to CV2 frame
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge failed: {e}")
            return
            
        fh, fw = frame.shape[:2]
        
        # Run YOLOv8 inference for person class (ID 0)
        results = self.model(frame, classes=[0], conf=self.conf_thresh, verbose=False)
        
        detections = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                detections.append((x1, y1, x2, y2, conf))
                
        # Run avoidance logic
        steer_cmd, dbg = self.ctrl.update(detections, fw, fh)
        
        # Publish control commands via Base class helper
        if self.steer_only_mode:
            # STATIONARY TEST: steer only, no drive/gear/park change
            # Vehicle stays in current gear/park state — safe for on-vehicle bench test
            self.publish_control_cmd(
                steer_target=steer_cmd,
                steer_speed=self.speed_dps,
                steer_en=True,
                speed_target=0.0,
                accel_target=0.0,
                drive_en=False,
                brake_en=False,
                brake_target=0.0,
                gear_en=False,
                park_en=False,
            )
        else:
            # FULL MODE: steer + forward motion
            self.publish_control_cmd(
                steer_target=steer_cmd,
                steer_speed=self.speed_dps,
                steer_en=True,
                speed_target=self.target_speed,
                accel_target=1.0,
                drive_en=True,
                brake_en=False,
                brake_target=0.0,
                gear_target=PixControlCmd.GEAR_TARGET_DRIVE,
                gear_en=True,
                park_target=PixControlCmd.PARK_TARGET_RELEASE,
                park_en=True
            )
        
        # Display overlay if enabled
        if not self.no_display:
            self.fps_deque.append(1.0 / max(time.time() - t0, 1e-6))
            
            # Simple visualization drawing
            for (x1, y1, x2, y2, conf) in detections:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"person {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
            # Draw center line
            cv2.line(frame, (fw // 2, 0), (fw // 2, fh), (255, 255, 0), 1)
            # HUD text
            cv2.putText(frame, f"STATE: {dbg['state']}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            cv2.putText(frame, f"OFFSET: {dbg['offset']:+.3f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"STEER: {dbg['ramp_pos']:+.1f} deg", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"FPS: {np.mean(self.fps_deque):.1f}", (fw - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            
            cv2.imshow("YOLO Person Avoidance", frame)
            cv2.waitKey(1)
            
    def destroy_node(self):
        if not self.no_display:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

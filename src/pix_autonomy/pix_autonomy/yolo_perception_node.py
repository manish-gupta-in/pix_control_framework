#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import os

try:
    from cv_bridge import CvBridge
    import cv2
    from ultralytics import YOLO
    HAS_DEPS = True
except ImportError:
    HAS_DEPS = False

class YOLOPerceptionNode(Node):
    """
    Real YOLO Perception Node using ultralytics.
    Subscribes to camera feed, detects persons, and outputs [distance, lateral_offset].
    """
    def __init__(self):
        super().__init__('yolo_perception')
        self.pub = self.create_publisher(Float32MultiArray, '/perception/obstacles', 10)
        
        if not HAS_DEPS:
            self.get_logger().error("Missing dependencies! Please install cv_bridge, opencv-python, and ultralytics.")
            self.get_logger().warn("Running in dummy mode.")
            self.timer = self.create_timer(1.0, self.dummy_loop)
            return

        self.bridge = CvBridge()
        
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('camera_topic', '/camera/right/image')
        
        yolo_path = self.get_parameter('model_path').value
        # If relative path fails, try finding it in the workspace root
        if not os.path.exists(yolo_path):
            fallback_path = os.path.join(os.getcwd(), 'yolov8n.pt')
            if os.path.exists(fallback_path):
                yolo_path = fallback_path
            else:
                self.get_logger().error(f"Could not find YOLO weights at {yolo_path}!")
                
        self.model = YOLO(yolo_path)
        self.get_logger().info(f"YOLO model loaded from {yolo_path}")
        
        topic = self.get_parameter('camera_topic').value
        self.sub = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"YOLO Perception Node listening on {topic}")
        
    def dummy_loop(self):
        msg = Float32MultiArray()
        msg.data = [999.0, 0.0]
        self.pub.publish(msg)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Run YOLO inference (classes=[0] means only detect persons)
            results = self.model(frame, classes=[0], conf=0.5, verbose=False)
            
            best_person = None
            max_area = 0
            
            if results and results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        best_person = (x1, y1, x2, y2)
                        
            out_msg = Float32MultiArray()
            
            if best_person:
                x1, y1, x2, y2 = best_person
                fh, fw = frame.shape[:2]
                cx = (x1 + x2) / 2.0
                
                # Normalised lateral offset [-1.0, 1.0] where 0 is center
                norm_offset = (cx - fw / 2.0) / (fw / 2.0)
                
                # Approximate distance using bounding box height 
                # (Assumes standard camera intrinsic and person height)
                h = y2 - y1
                # Rough heuristic: a person at 1 meter might be ~500px tall in this camera
                distance = 500.0 / (h + 1e-5) 
                
                out_msg.data = [float(distance), float(norm_offset)]
                
                # Draw bounding box for visualization
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {distance:.2f}m", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            else:
                # No person detected
                out_msg.data = [999.0, 0.0]
                
            self.pub.publish(out_msg)
            
            # Display the camera window
            # Resize it so it fits nicely on screen
            display_frame = cv2.resize(frame, (800, 600))
            cv2.imshow("YOLO Perception", display_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

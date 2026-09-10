import pytest
import rclpy

def test_imports():
    """Test that all nodes can be imported."""
    from pix_autonomy.aeb_node import AEBSystemNode
    from pix_autonomy.control_arbitrator_node import ControlArbitratorNode
    from pix_autonomy.gnss_waypoint_follower import GNSSWaypointFollower
    from pix_autonomy.mpc_planner_node import MPCPlannerNode
    from pix_autonomy.yolo_perception_node import YOLOPerceptionNode
    from pix_autonomy.lateral_avoidance_node import LateralAvoidancePlanner
    from pix_autonomy.straight_drive_node import StraightDriveNode
    
    # Just passing the imports means syntax is valid
    assert True

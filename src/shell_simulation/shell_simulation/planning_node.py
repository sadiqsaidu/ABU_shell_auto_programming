#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
import math

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Parameters
        self.declare_parameter('waypoint_achievement_threshold', 3.0)  # meters
        self.waypoint_threshold = self.get_parameter('waypoint_achievement_threshold').get_parameter_value().double_value
        self.declare_parameter('target_frame_id', 'odom') # Or 'map', check CARLA odometry frame
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value
        
        # List of ALL 15 waypoints (start + 14 targets)
        # Ensure these are in the order you want to visit them for this simple planner
        # For a more advanced planner, you'd implement TSP (Traveling Salesperson Problem) logic here
        self.waypoints = [
            Point(x=280.363739, y=-129.306351, z=0.101746), # Start point (also first goal)
            Point(x=334.949799, y=-161.106171, z=0.001736),
            Point(x=339.100037, y=-258.568939, z=0.001679),
            Point(x=396.295319, y=-183.195740, z=0.001678),
            Point(x=267.657074, y=-1.983160,   z=0.001678),
            Point(x=153.868896, y=-26.115866,  z=0.001678),
            Point(x=290.515564, y=-56.175072,  z=0.001677),
            Point(x=92.325722,  y=-86.063644,  z=0.001677),
            Point(x=88.384346,  y=-287.468567, z=0.001728),
            Point(x=177.594101, y=-326.386902, z=0.001677),
            Point(x=-1.646942,  y=-197.501282, z=0.001555),
            Point(x=59.701321,  y=-1.970804,   z=0.001467),
            Point(x=122.100121, y=-55.142044,  z=0.001596),
            Point(x=161.030975, y=-129.313187, z=0.001679),
            Point(x=184.758713, y=-199.424271, z=0.001680)
        ]
        self.current_waypoint_index = 0
        self.all_waypoints_reached = False

        # Subscription to Odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10)
        
        # Publisher for Current Target Waypoint
        self.target_waypoint_publisher = self.create_publisher(PointStamped, '/current_target_waypoint', 10)
        
        self.get_logger().info(f"Planning Node started. Number of waypoints: {len(self.waypoints)}")
        self.publish_current_target() # Publish the first waypoint initially

    def publish_current_target(self):
        if self.current_waypoint_index < len(self.waypoints):
            target_point_msg = PointStamped()
            target_point_msg.header.stamp = self.get_clock().now().to_msg()
            target_point_msg.header.frame_id = self.target_frame_id 
            target_point_msg.point = self.waypoints[self.current_waypoint_index]
            self.target_waypoint_publisher.publish(target_point_msg)
            self.get_logger().info(f"Publishing waypoint {self.current_waypoint_index}: {target_point_msg.point.x:.2f}, {target_point_msg.point.y:.2f}")
        else:
            if not self.all_waypoints_reached:
                self.get_logger().info("All waypoints reached!")
                self.all_waypoints_reached = True

    def odom_callback(self, msg):
        if self.all_waypoints_reached:
            return

        current_position = msg.pose.pose.position
        target_waypoint = self.waypoints[self.current_waypoint_index]
        
        distance = math.sqrt(
            (target_waypoint.x - current_position.x)**2 +
            (target_waypoint.y - current_position.y)**2 
            # Not considering Z for waypoint achievement in this simple version
        )
        
        if distance < self.waypoint_threshold:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
            else:
                self.all_waypoints_reached = True # Mark all as reached
            
            self.publish_current_target() # Publish next or indicate completion by behavior
        else:
            # Periodically republish the current target if not reached,
            # useful if controller starts late or needs continuous target
             self.publish_current_target()


def main(args=None):
    rclpy.init(args=args)
    planning_node = PlanningNode()
    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
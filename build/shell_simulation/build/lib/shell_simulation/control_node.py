#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 as RosFloat32 # For speedometer
from std_msgs.msg import Float64 as RosFloat64 # For control commands
from std_msgs.msg import String as RosString
from std_msgs.msg import Bool as RosBool
from geometry_msgs.msg import PointStamped
import math
import tf_transformations # sudo apt-get install ros-humble-tf-transformations

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Parameters (tune these!)
        self.declare_parameter('kp_steer', 0.8) 
        self.declare_parameter('kp_throttle', 0.3)
        self.declare_parameter('target_speed_mps', 8.0)  # m/s (e.g., ~18 mph)
        self.declare_parameter('speed_limit_mps', 13.0) # Approx 29 mph (30mph limit is 13.41 m/s)
        self.declare_parameter('waypoint_brake_distance', 5.0) # meters, distance to start braking for waypoint
        self.declare_parameter('control_loop_period', 0.05) # seconds (20 Hz)

        self.kp_steer = self.get_parameter('kp_steer').get_parameter_value().double_value
        self.kp_throttle = self.get_parameter('kp_throttle').get_parameter_value().double_value
        self.target_speed_mps = self.get_parameter('target_speed_mps').get_parameter_value().double_value
        self.speed_limit_mps = self.get_parameter('speed_limit_mps').get_parameter_value().double_value
        self.waypoint_brake_distance = self.get_parameter('waypoint_brake_distance').get_parameter_value().double_value
        
        # Vehicle state variables
        self.current_odometry = None
        self.current_speed_mps = 0.0
        self.target_waypoint = None
        self.obstacle_alert = False
        self.initial_commands_sent = False

        # Publishers for Vehicle Control
        self.throttle_pub = self.create_publisher(RosFloat64, '/throttle_command', 10)
        self.steer_pub = self.create_publisher(RosFloat64, '/steering_command', 10)
        self.brake_pub = self.create_publisher(RosFloat64, '/brake_command', 10)
        self.gear_pub = self.create_publisher(RosString, '/gear_command', 10) # QoS needs to be reliable for latched-like
        self.handbrake_pub = self.create_publisher(RosBool, '/handbrake_command', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/carla/ego_vehicle/odometry', self.odom_callback, 10)
        self.speed_sub = self.create_subscription(
            RosFloat32, '/carla/ego_vehicle/speedometer', self.speed_callback, 10)
        self.target_wp_sub = self.create_subscription(
            PointStamped, '/current_target_waypoint', self.target_waypoint_callback, 10)
        self.obstacle_alert_sub = self.create_subscription(
            RosBool, '/obstacle_alert', self.obstacle_alert_callback, 10)

        # Control loop timer
        self.timer = self.create_timer(self.get_parameter('control_loop_period').get_parameter_value().double_value, self.control_loop)
        
        self.get_logger().info("Control Node started.")

    def send_initial_commands(self):
        if not self.initial_commands_sent:
            gear_msg = RosString()
            gear_msg.data = "forward"
            self.gear_pub.publish(gear_msg)

            handbrake_msg = RosBool()
            handbrake_msg.data = False
            self.handbrake_pub.publish(handbrake_msg)
            
            self.initial_commands_sent = True
            self.get_logger().info("Initial gear and handbrake commands sent.")

    def odom_callback(self, msg):
        self.current_odometry = msg

    def speed_callback(self, msg):
        self.current_speed_mps = msg.data # Assuming this is in m/s from CARLA

    def target_waypoint_callback(self, msg):
        self.target_waypoint = msg.point

    def obstacle_alert_callback(self, msg):
        if self.obstacle_alert != msg.data: # Log change in state
             self.get_logger().info(f"Obstacle alert changed to: {msg.data}")
        self.obstacle_alert = msg.data

    def control_loop(self):
        if not self.initial_commands_sent:
            self.send_initial_commands()
            # Return early on the first few cycles to ensure CARLA has processed initial commands
            # and we have received some sensor data.
            if not self.current_odometry or self.target_waypoint is None:
                 self.get_logger().info("Waiting for odometry and first target waypoint...")
                 # Publish zero commands to be safe
                 self.throttle_pub.publish(RosFloat64(data=0.0))
                 self.steer_pub.publish(RosFloat64(data=0.0))
                 self.brake_pub.publish(RosFloat64(data=0.0))
                 return


        if self.obstacle_alert:
            self.get_logger().warn("Obstacle Detected! Applying full brakes.")
            self.throttle_pub.publish(RosFloat64(data=0.0))
            self.steer_pub.publish(RosFloat64(data=0.0)) # Or maintain current steering
            self.brake_pub.publish(RosFloat64(data=1.0)) # Full brake
            return

        if self.current_odometry is None or self.target_waypoint is None:
            # self.get_logger().info("Waiting for odometry or target waypoint...")
            # Publish zero commands to be safe if state is unknown after initial wait
            self.throttle_pub.publish(RosFloat64(data=0.0))
            self.steer_pub.publish(RosFloat64(data=0.0))
            self.brake_pub.publish(RosFloat64(data=0.0))
            return

        # Current vehicle state
        current_pos = self.current_odometry.pose.pose.position
        orientation_q = self.current_odometry.pose.pose.orientation
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, current_yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        # Calculate heading error
        dx = self.target_waypoint.x - current_pos.x
        dy = self.target_waypoint.y - current_pos.y
        target_yaw = math.atan2(dy, dx)
        
        heading_error = target_yaw - current_yaw
        # Normalize heading error to [-pi, pi]
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Steering control (P-controller)
        steer_cmd_val = self.kp_steer * heading_error
        steer_cmd_val = max(-1.0, min(1.0, steer_cmd_val)) # Clamp to [-1.0, 1.0]
        
        # Throttle and Brake Control
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        throttle_cmd_val = 0.0
        brake_cmd_val = 0.0

        # Speed limit enforcement
        if self.current_speed_mps > self.speed_limit_mps:
            # self.get_logger().warn(f"Speed limit ({self.speed_limit_mps:.1f}m/s) exceeded: {self.current_speed_mps:.1f}m/s. Applying brakes.")
            throttle_cmd_val = 0.0
            brake_cmd_val = 0.5 # Moderate braking for speed limit
        elif distance_to_target < self.waypoint_brake_distance:
            # self.get_logger().info(f"Approaching waypoint (dist: {distance_to_target:.1f}m). Reducing speed.")
            # Gentle braking as we approach the waypoint, proportional to how close we are
            brake_cmd_val = 0.2 + 0.8 * (1.0 - (distance_to_target / self.waypoint_brake_distance))
            brake_cmd_val = max(0.0, min(1.0, brake_cmd_val))
            throttle_cmd_val = 0.1 # Crawl speed
            if distance_to_target < 1.0 : # Very close, aim to stop
                 throttle_cmd_val = 0.0
                 brake_cmd_val = 0.8
        else:
            # P-controller for speed, only if reasonably aligned (e.g. heading_error is small)
            if abs(heading_error) < math.radians(30): # Only apply throttle if somewhat aligned
                speed_error = self.target_speed_mps - self.current_speed_mps
                throttle_cmd_val = self.kp_throttle * speed_error
            else: # If not aligned, focus on steering, reduce throttle
                throttle_cmd_val = 0.1 # Low throttle while turning significantly
            
        throttle_cmd_val = max(0.0, min(1.0, throttle_cmd_val)) # Clamp throttle
        brake_cmd_val = max(0.0, min(1.0, brake_cmd_val)) # Clamp brake

        # Publish commands
        self.throttle_pub.publish(RosFloat64(data=throttle_cmd_val))
        self.steer_pub.publish(RosFloat64(data=steer_cmd_val))
        self.brake_pub.publish(RosFloat64(data=brake_cmd_val))

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly shut down brake/throttle
        control_node.throttle_pub.publish(RosFloat64(data=0.0))
        control_node.brake_pub.publish(RosFloat64(data=1.0)) # Apply brake on exit
        control_node.get_logger().info("Control node shutting down, applying brakes.")
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
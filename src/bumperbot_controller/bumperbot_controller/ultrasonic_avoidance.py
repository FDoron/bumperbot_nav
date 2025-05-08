#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import Twist

class UltrasonicAvoidance(Node):
    def __init__(self):
        super().__init__("ultrasonic_avoidance")

        # Subscribe to ultrasonic sensor topic
        self.subscription = self.create_subscription( LaserScan, "/ultrasonic/out", self.ultrasonic_callback, 10)

        # Publisher for safe velocity command
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)

        # Parameters
        self.slow_down_distance = 0.5  # Start slowing down when object is within 0.5m
        self.stop_distance = 0.2       # Stop forward motion when object is within 0.2m
        self.max_speed = 0.2           # Default forward speed

        self.get_logger().info("Ultrasonic Avoidance Node Initialized")

    def ultrasonic_callback(self, msg):
        distance = float(msg.ranges[0]) 
        self.get_logger().info(f"distance: {distance}m")
        twist = Twist()

        if distance < self.stop_distance:
            # Stop forward motion but allow reversing
            twist.linear.x = min(0.0, twist.linear.x)  
            self.get_logger().warn(f"Obstacle too close! Stopping forward movement. Distance: {distance:.2f}m")
        elif distance < self.slow_down_distance:
            # Reduce speed based on proximity
            twist.linear.x = self.max_speed * (distance / self.slow_down_distance)
            self.get_logger().info(f"Slowing down. Distance: {distance:.2f}m, Speed: {twist.linear.x:.2f}m/s")
        # else:
            # Move normally
            # twist.linear.x = self.max_speed

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

class RangeToFloat32(Node):
    def __init__(self):
        super().__init__('range_to_float32')
        # Subscribe to the remapped topic from the launch file
        self.subscription = self.create_subscription(
            Range,
            '/ultrasonic',  # This will be remapped to /ultrasonic/out
            self.listener_callback,
            10
        )
        # Publish to a different topic
        self.publisher_ = self.create_publisher(
            Float32, 
            '/ultrasonic/float',  # Changed to avoid conflict
            10
        )

    def listener_callback(self, data):
        float_msg = Float32()
        float_msg.data = data.range
        self.publisher_.publish(float_msg)

def main(args=None):
    rclpy.init(args=args)
    range_to_float32 = RangeToFloat32()
    rclpy.spin(range_to_float32)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class FrameVerifier(Node):
    def __init__(self):
        super().__init__('frame_verifier')
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a timer to check transforms periodically
        self.timer = self.create_timer(5.0, self.check_transforms)
    
    def check_transforms(self):
        try:
            # Check transforms between key frames
            frames_to_check = [
                ('map', 'odom'),
                ('odom', 'base_link'),
                ('map', 'base_link')
            ]
            
            for source, target in frames_to_check:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target, source, 
                        rclpy.time.Time(), 
                        rclpy.duration.Duration(seconds=1.0)
                    )
                    self.get_logger().info(f'Transform {source} → {target}: OK')
                    self.log_transform_details(transform)
                except Exception as e:
                    self.get_logger().error(f'Failed to get transform {source} → {target}: {str(e)}')
        
        except Exception as e:
            self.get_logger().error(f'Error checking transforms: {str(e)}')
    
    def log_transform_details(self, transform):
        # Log detailed transform information
        self.get_logger().info(f'Translation: x={transform.transform.translation.x}, '
                                f'y={transform.transform.translation.y}, '
                                f'z={transform.transform.translation.z}')
        self.get_logger().info(f'Rotation: x={transform.transform.rotation.x}, '
                                f'y={transform.transform.rotation.y}, '
                                f'z={transform.transform.rotation.z}, '
                                f'w={transform.transform.rotation.w}')

def main(args=None):
    rclpy.init(args=args)
    node = FrameVerifier()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
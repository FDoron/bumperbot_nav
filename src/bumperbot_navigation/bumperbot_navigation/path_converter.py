import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathConverter(Node):
    def __init__(self):
        super().__init__('path_converter')
        self.f2c_subscription = self.create_subscription(
            Path, 
            '/fields2cover/path', 
            self.convert_path, 
            10
        )
        self.plan_publisher = self.create_publisher(Path, '/plan', 10)
        self.get_logger().info('Path converter node initialized')

    def convert_path(self, f2c_path):
        nav2_path = Path()
        nav2_path.header = f2c_path.header
        nav2_path.header.frame_id = 'map'
        nav2_path.poses = f2c_path.poses
        self.plan_publisher.publish(nav2_path)

def main(args=None):
    rclpy.init(args=args)
    path_converter = PathConverter()
    rclpy.spin(path_converter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
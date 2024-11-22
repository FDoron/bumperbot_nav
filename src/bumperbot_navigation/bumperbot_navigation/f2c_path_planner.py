import rclpy
from rclpy.node import Node
import fields2cover as f2c
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class F2CPathPlanner(Node):
    def __init__(self):
        super().__init__('f2c_path_planner')
        
        # Create visualization publisher
        self.viz_pub = self.create_publisher(Marker, 'f2c_path', 10)
        
        # Create field using Field class
        field = f2c.Field()
        # Create field boundary
        points = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0)
        ]
        
        # Create field boundary
        field.polygon = points
        
        # Set swath parameters
        swath = f2c.SwathGenerator()
        swath.width = 2.0  # meters
        swath.heading = 0.0  # radians
        
        # Generate path
        path = swath.generateSwaths(field)
        
        # Visualize path
        self.publish_path_marker(path)
        self.get_logger().info('Path generated and published!')

    def publish_path_marker(self, path):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        
        for swath in path:
            for point in swath:
                p = Point()
                p.x = point[0]  # x coordinate
                p.y = point[1]  # y coordinate
                p.z = 0.0
                marker.points.append(p)
            
        self.viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = F2CPathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
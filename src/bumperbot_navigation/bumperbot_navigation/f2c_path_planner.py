import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import fields2cover as f2c
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
import tf_transformations


class F2CPathPlanner(Node):
    def __init__(self):
        super().__init__('f2c_path_planner')

        # Create QoS profile
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers for both visualization and navigation
        self.viz_pub = self.create_publisher(Marker, 'f2c_path_viz', qos_profile)
        self.path_pub = self.create_publisher(Path, '/fields2cover/path', qos_profile)
       
       
        # Create field using Field class
        field = f2c.Field()
        # Create field boundary
        coordinates = [
            [0.0, 0.0],
            [0.0, 5.0],
            [5.0, 5.0],
            [5.0, 0.0],
            [0.0, 0.0]
        ]

        points = f2c.VectorPoint()
        for x, y in coordinates:
            point = f2c.Point(x, y)
            points.push_back(point)
       
        # Create LinearRing from points
        field_ring = f2c.LinearRing(points)

        cell = f2c.Cell()
        cell.addRing(field_ring)

        cells = f2c.Cells()
        cells.addGeometry(cell)

        field = f2c.Field(cells)

        # Transform into UTM to work in meters
        # f2c.Transform.transformToUTM(field)

        robot = f2c.Robot(0.3, 0.4)
        const_hl = f2c.HG_Const_gen()
        no_hl = const_hl.generateHeadlands(field.field, 3.0 * robot.robot_width)
        bf = f2c.SG_BruteForce()
        swaths = bf.generateSwaths(math.pi, robot.op_width, no_hl.getGeometry(0))
        boustrophedon_sorter = f2c.RP_Boustrophedon()
        swaths = boustrophedon_sorter.genSortedSwaths(swaths)

        robot.setMinRadius(0.1)
        path_planner = f2c.PP_PathPlanning()
        dubins = f2c.PP_DubinsCurves()
        path = path_planner.searchBestPath(robot, swaths, dubins)
        
        self.publish_paths(path)
        self.get_logger().info('Path generated and published!')

    def calculate_orientation(self, current_point, next_point):
        # Calculate yaw angle between current and next points
        dx = next_point.x - current_point.x
        dy = next_point.y - current_point.y
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return q

    def publish_paths(self, path):
        # Create messages
        marker = Marker()
        nav_path = Path()
        
        # Set headers
        now = self.get_clock().now().to_msg()
        marker.header.stamp = now
        marker.header.frame_id = "map"
        nav_path.header.stamp = now
        nav_path.header.frame_id = "map"
        
        # Set marker properties
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.lifetime = Duration(sec=0, nanosec=0)
        
        path_size = path.size()
        
        # Iterate through path
        for i in range(path_size):
            state = path.states[i]
            point = state.point
            
            # Create point for marker
            p = Point()
            p.x = point.getX()
            p.y = point.getY()
            p.z = point.getZ()
            marker.points.append(p)
            
            # Create pose for nav path
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position = p
            
            # Calculate orientation if not last point
            if i < path_size - 1:
                next_state = path.states[i + 1]
                next_point = Point()
                next_point.x = next_state.point.getX()
                next_point.y = next_state.point.getY()
                q = self.calculate_orientation(p, next_point)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            else:
                # For last point, use previous orientation
                if nav_path.poses:
                    pose.pose.orientation = nav_path.poses[-1].pose.orientation
                else:
                    pose.pose.orientation.w = 1.0
            
            nav_path.poses.append(pose)
        
        # Publish both messages
        self.viz_pub.publish(marker)
        self.path_pub.publish(nav_path)


def main(args=None):
    rclpy.init(args=args)
    node = F2CPathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
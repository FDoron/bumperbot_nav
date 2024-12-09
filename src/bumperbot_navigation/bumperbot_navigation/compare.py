#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PathConverter(Node):
    def __init__(self):
        super().__init__('path_converter')
        
        # Create QoS profile for reliable communication
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.f2c_subscription = self.create_subscription(
            Path, 
            '/fields2cover/path', 
            self.convert_path, 
            reliable_qos
        )
        
        self.plan_publisher = self.create_publisher(
            Path, 
            '/plan', 
            reliable_qos
        )
        
        self.get_logger().info('Path converter node initialized')

    def calculate_orientation(self, current_pose, next_pose):
        # Calculate yaw angle between current and next pose
        dx = next_pose.pose.position.x - current_pose.pose.position.x
        dy = next_pose.pose.position.y - current_pose.pose.position.y
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        quaternion = Quaternion()
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
        
        return quaternion

    def convert_path(self, f2c_path):
        self.get_logger().info(f'Received F2C path with {len(f2c_path.poses)} poses')
        
        if len(f2c_path.poses) == 0:
            self.get_logger().warn('Received empty path from fields2cover')
            return
            
        nav2_path = Path()
        nav2_path.header = f2c_path.header
        nav2_path.header.frame_id = 'map'  # Ensure we're using the map frame
        nav2_path.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
        
        # Process poses and add orientations
        poses = []
        for i in range(len(f2c_path.poses)):
            pose = PoseStamped()
            pose.header = nav2_path.header
            pose.pose.position = f2c_path.poses[i].pose.position
            
            # Calculate orientation (except for the last point)
            if i < len(f2c_path.poses) - 1:
                pose.pose.orientation = self.calculate_orientation(
                    f2c_path.poses[i],
                    f2c_path.poses[i + 1]
                )
            else:
                # For the last point, use the same orientation as the previous point
                pose.pose.orientation = poses[-1].pose.orientation if poses else Quaternion(w=1.0)
            
            poses.append(pose)
            
            # Debug output for each pose
            self.get_logger().debug(
                f'Pose {i}: Position(x:{pose.pose.position.x:.2f}, '
                f'y:{pose.pose.position.y:.2f}, z:{pose.pose.position.z:.2f}), '
                f'Orientation(x:{pose.pose.orientation.x:.2f}, y:{pose.pose.orientation.y:.2f}, '
                f'z:{pose.pose.orientation.z:.2f}, w:{pose.pose.orientation.w:.2f})'
            )
        
        nav2_path.poses = poses
        self.plan_publisher.publish(nav2_path)
        self.get_logger().info(f'Published Nav2 path with {len(poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    path_converter = PathConverter()
    
    try:
        rclpy.spin(path_converter)
    except Exception as e:
        path_converter.get_logger().error(f'Error in path converter: {str(e)}')
    finally:
        path_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
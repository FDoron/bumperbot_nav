#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time

class Nav2TwistRelay(Node):
    def __init__(self):
        super().__init__('nav2_twist_relay')
        
        self.cmd_pub = self.create_publisher(Twist, '/nav_vel', 10)
        
        # Detailed action client setup
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Extensive waiting and logging for action server
        self.get_logger().info('Waiting for navigation action server...')
        if not self.navigate_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server failed to appear after 10 seconds')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.create_subscription(Path, '/plan', self.execute_path, qos_profile)
        
        self.get_logger().info('Nav2 Twist Relay initialized')
        
    def execute_path(self, path):
        try:
            if not path.poses:
                self.get_logger().warn('Received empty path')
                return
            
            self.get_logger().info(f'Received path with {len(path.poses)} poses')
            
            # Comprehensive action server readiness check
            if not self.navigate_action_client.server_is_ready():
                self.get_logger().error('Navigation action server not ready')
                
                # Detailed diagnostics
                self.get_logger().info(f'Action client connected: {self.navigate_action_client.is_client_connected()}')
                self.get_logger().info(f'Server wait timeout: {self.navigate_action_client.server_wait_timeout}')
                
                # Attempt to reconnect
                try:
                    self.navigate_action_client.wait_for_server(timeout_sec=5.0)
                except Exception as wait_error:
                    self.get_logger().error(f'Failed to wait for server: {wait_error}')
                return
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = path.poses[-1]  # Final pose as goal
            
            send_goal_future = self.navigate_action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error in execute_path: {str(e)}')
        
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Navigation goal rejected')
                return
            
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error in goal_response_callback: {str(e)}')
        
    def get_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation to goal succeeded!')
            else:
                self.get_logger().warn(f'Navigation failed with status: {status}')
                
        except Exception as e:
            self.get_logger().error(f'Error in get_result_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2TwistRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
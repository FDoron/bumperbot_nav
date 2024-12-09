import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped  # Correctly importing PoseStamped

class MultiWaypointFollower(Node):
    def __init__(self):
        super().__init__('multi_waypoint_follower')
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        self.waypoints = []  # List to hold waypoints

        # Create a timer to periodically send waypoints for testing purposes
        self.timer = self.create_timer(1.0, self.send_waypoint)

        # Initialize waypoints (mock data)
        self.initialize_waypoints()

    def initialize_waypoints(self):
        # Add some mock waypoints (you can replace this with actual data)
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = i * 1.0  # Change these values as needed for your scenario
            pose.pose.position.y = i * 1.5
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            self.waypoints.append(pose)

        self.get_logger().info('Initialized waypoints.')

    def send_waypoint(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints available.')
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add all waypoints to the path message
        for waypoint in self.waypoints:
            path_msg.poses.append(waypoint)

        # Send the path to the action server
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return
        
        self.get_logger().info('Goal accepted.')

def main(args=None):
    rclpy.init(args=args)
    
    multi_waypoint_follower = MultiWaypointFollower()
    
    rclpy.spin(multi_waypoint_follower)

    multi_waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
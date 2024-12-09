import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        # Configurable parameters
        self.declare_parameter('execution_delay', 5.0)  # Default 5 second delay
        self.declare_parameter('max_path_age', 60.0)   # Max age of path before considering it stale
        
        self.execution_delay = self.get_parameter('execution_delay').value
        self.max_path_age = self.get_parameter('max_path_age').value
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        self.path_subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            reliable_qos
        )
        
        self.latest_path = None
        self.latest_path_timestamp = None
        
        # Timer for delayed execution
        self.execution_timer = None

    def path_callback(self, msg):
        # Detailed path validation and logging
        if len(msg.poses) == 0:
            self.get_logger().error('Empty path received!')
            return
        
        # Verify path frame
        if msg.header.frame_id == '':
            self.get_logger().warn('Path has no frame_id specified!')
        else:
            self.get_logger().info(f'Path frame_id: {msg.header.frame_id}')
        
        # Detailed path information
        self.get_logger().info(f'Path contains {len(msg.poses)} poses')
        
        # Log first and last pose details with frame information
        first_pose = msg.poses[0]
        last_pose = msg.poses[-1]
        
        self.get_logger().info(f'First pose - Frame: {first_pose.header.frame_id}')
        self.get_logger().info(f'First pose - x: {first_pose.pose.position.x}, y: {first_pose.pose.position.y}')
        
        self.get_logger().info(f'Last pose - Frame: {last_pose.header.frame_id}')
        self.get_logger().info(f'Last pose - x: {last_pose.pose.position.x}, y: {last_pose.pose.position.y}')
        
        # Store the path
        self.latest_path = msg
        self.latest_path_timestamp = self.get_clock().now()

    def log_path_details(self, msg):
        """Log details about the received path"""
        self.get_logger().info(f'Received path with {len(msg.poses)} poses')
        if len(msg.poses) > 0:
            first_pose = msg.poses[0]
            last_pose = msg.poses[-1]
            self.get_logger().info(f'Path starts at x: {first_pose.pose.position.x}, y: {first_pose.pose.position.y}')
            self.get_logger().info(f'Path ends at x: {last_pose.pose.position.x}, y: {last_pose.pose.position.y}')

    def prepare_path_execution(self):
        """Prepare and validate path before execution"""
        # Check if path is still valid
        if not self.is_path_valid():
            self.get_logger().warn('Path is no longer valid. Cancelling execution.')
            return
        
        # Cancel the timer
        if self.execution_timer:
            self.execution_timer.cancel()
        
        # Attempt to send path
        self.send_path()

    def is_path_valid(self):
        """Check if the path is still valid"""
        if self.latest_path is None:
            return False
        
        # Check path age
        current_time = self.get_clock().now()
        path_age = current_time - self.latest_path_timestamp
        
        if path_age.nanoseconds / 1e9 > self.max_path_age:
            self.get_logger().warn(f'Path is {path_age.nanoseconds / 1e9:.2f} seconds old, exceeding max age of {self.max_path_age} seconds')
            return False
        
        return True

    def send_path(self):
        """Send path to Nav2 action server with enhanced error handling"""
        try:
            # Validate action client
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Failed to connect to follow_path action server')
                return
            
            # Prepare goal message
            goal_msg = FollowPath.Goal()
            goal_msg.path = self.latest_path
            
            # Send goal with timeout
            send_goal_future = self.action_client.send_goal_async(
                goal_msg, 
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
        
        except Exception as e:
            self.get_logger().error(f'Error sending path: {str(e)}')

    def goal_response_callback(self, future):
        """Handle goal response with improved error logging"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Path following goal was rejected')
                return
            
            self.get_logger().info('Path following goal accepted')
            
            # Get result future with timeout
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)
        
        except Exception as e:
            self.get_logger().error(f'Error in goal response: {str(e)}')

    def get_result_callback(self, future):
        try:
            # Get the entire result
            result_wrapper = future.result()
            
            # Log full result details
            self.get_logger().error(f'Result wrapper type: {type(result_wrapper)}')
            
            # Attempt to extract result
            if hasattr(result_wrapper, 'result'):
                result = result_wrapper.result
                self.get_logger().error(f'Result type: {type(result)}')
                self.get_logger().error(f'Result content: {result}')
                
                # Check if it's an Empty message
                if result.__class__.__name__ == 'Empty':
                    self.get_logger().error('Received an Empty result message')
                    
                # Try to get any status or code
                if hasattr(result, 'status'):
                    self.get_logger().error(f'Result status: {result.status}')
            else:
                self.get_logger().error('No result attribute found in the result wrapper')
        
        except Exception as e:
            self.get_logger().error(f'Exception in result callback: {str(e)}')
            import traceback
            traceback.print_exc()

    def feedback_callback(self, feedback_msg):
        """Handle feedback with logging"""
        try:
            feedback = feedback_msg.feedback
            self.get_logger().debug(f'Path following feedback: {feedback}')
        except Exception as e:
            self.get_logger().error(f'Error processing feedback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    path_follower = None
    
    try:
        path_follower = PathFollower()
        rclpy.spin(path_follower)
    
    except Exception as e:
        print(f'Unhandled exception: {e}')
    
    finally:
        if path_follower:
            path_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
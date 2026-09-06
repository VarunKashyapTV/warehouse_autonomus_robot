import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException   
from action_msgs.msg import GoalStatus

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.get_logger().info('Mission Controller started')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Gates the input thread: set() = safe to prompt for next waypoint,
        # clear() = a goal is currently in flight, input thread must wait.
        # Starts set() so the very first prompt isn't blocked.
        self.goal_done_event = threading.Event()
        self.goal_done_event.set()

        # Start background thread for terminal input to avoid blocking rclpy.spin()
        self.input_thread = threading.Thread(target=self.terminal_waypoint_loop, daemon=True)
        self.input_thread.start()

    def terminal_waypoint_loop(self):
        """Background thread loop that listens for terminal input continuous stream."""
        # Wait for Nav2 action server to become active
        self.action_client.wait_for_server()
        self.get_logger().info("Nav2 Action Server connected. Ready for waypoints.")
        
        while rclpy.ok():
            # Block here until the previous goal (if any) has finished.
            # This guarantees only one input() call is ever outstanding.
            self.goal_done_event.wait()

            try:
                raw = input("\nEnter next waypoint as 'x,y' (or 'q' to quit): ").strip()
                if raw.lower() == 'q':
                    self.get_logger().info("Mission ended by user")
                    rclpy.shutdown()
                    sys.exit(0)
                
                x_str, y_str = raw.split(',')
                x, y = float(x_str), float(y_str)

                # About to send a goal — close the gate until it completes.
                self.goal_done_event.clear()

                # Send waypoint using your existing pipeline
                self.process_waypoint_input(x, y)
                
            except ValueError:
                print("Invalid format! Expected format: x,y  e.g. 1.5,0.0")
                # No goal was sent, so don't leave the gate closed.
                self.goal_done_event.set()
            except (KeyboardInterrupt, EOFError):
                break

    def process_waypoint_input(self, x: float, y: float):
        """Constructs PoseStamped and sends goal through existing action client."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        
        self.send_navigation_goal(pose)


    def send_navigation_goal(self, pose: PoseStamped):
        self.get_logger().info(f'Sending navigation goal: ({pose.pose.position.x}, {pose.pose.position.y})')
        self.action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            # Goal never started, so nothing will ever call get_navigation_result_callback
            # for it. Re-open the gate here or the input thread hangs forever.
            self.goal_done_event.set()
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_navigation_result_callback)

    def get_navigation_result_callback(self, future):
        result = future.result()
        status = result.status
    
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation ABORTED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation CANCELED')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')

        # Goal is fully resolved (success or failure) — safe to prompt for the next one.
        self.goal_done_event.set()
    

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
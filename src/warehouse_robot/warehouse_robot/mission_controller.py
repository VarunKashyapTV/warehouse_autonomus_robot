import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.get_logger().info('Mission Controller started')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.request_waypoint_from_terminal()

    def request_waypoint_from_terminal(self):
        while True:
            try:
                raw = input("Enter next waypoint as 'x,y' (or 'q' to quit): ").strip()
                if raw.lower() == 'q':
                    self.get_logger().info("mission ended by user")
                    rclpy.shutdown()
                    return
                x_str, y_str = raw.split(',')
                x, y = float(x_str), float(y_str)
                break
            except ValueError:
                print("Invalid format, expected: x,y  e.g. 1.5,0.0")

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        self.send_navigation_goal(pose)

    def send_navigation_goal(self, pose: PoseStamped):
        self.get_logger().info(f'Sending navigation goal: {pose}')
        self.action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.request_waypoint_from_terminal()
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_navigation_result_callback)

    def get_navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result received: {result}')
        self.request_waypoint_from_terminal()

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
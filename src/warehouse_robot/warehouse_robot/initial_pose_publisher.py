import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    """Publishes /initialpose once, stamped with real sim time, after the sim is ready."""

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.ready_ticks = 0
        self.done = False
        # Poll on the wall clock (steady), so this works even before sim time starts moving.
        self.timer = self.create_timer(
            1.0, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Waiting for robot (/odom), AMCL (/initialpose subscriber) and sim clock...')

    def tick(self):
        if self.done:
            return

        now = self.get_clock().now()  # sim time, because use_sim_time=True
        ready = (
            self.count_publishers('/odom') > 0           # robot really exists in Gazebo
            and self.pub.get_subscription_count() > 0    # AMCL is listening
            and now.nanoseconds > 0                      # sim clock has started
        )
        self.ready_ticks = self.ready_ticks + 1 if ready else 0

        # Need 3 ready polls in a row (~2 s of stability), same as the old `sleep 2`.
        if self.ready_ticks < 3:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = now.to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)

        stamp = msg.header.stamp
        self.get_logger().info(
            f'Initial pose (0.0, 0.0) published with sim-time stamp {stamp.sec}.{stamp.nanosec:09d}')
        self.done = True


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        # a few more spins so the reliable message is flushed before exiting
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

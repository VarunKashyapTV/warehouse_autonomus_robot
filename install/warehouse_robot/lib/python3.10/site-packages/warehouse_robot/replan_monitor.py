import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class ReplanMonitor(Node):
    def __init__(self):
        super().__init__("replan_monitor")
        self.get_logger().info("replan monitor node has started")

        self.subscription = self.create_subscription(Path, "/plan", self.pose_callback, 10)

        self.previous_path = None          # list of (x, y) from the last /plan message
        self.is_first_message = True

        # --- tunable parameters ---
        self.point_threshold = 0.1         # meters: a single point counts as "moved" above this
        self.cluster_size = 3              # how many *consecutive* moved points = a real obstacle event
        self.cooldown_sec = 3.0            # don't re-log within this many seconds of the last detection

        self.last_detection_time = None

    def pose_callback(self, msg: Path):
        current_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        if self.is_first_message:
            self.get_logger().info(f"initial plan received with {len(current_pts)} poses")
            self.is_first_message = False
            self.previous_path = current_pts
            return

        if not current_pts or not self.previous_path:
            self.previous_path = current_pts
            return

        max_dev, moved_flags = self.compute_deviation(self.previous_path, current_pts)

        if self.has_cluster(moved_flags) and self.cooldown_elapsed():
            self.get_logger().info(
                f"REPLANNING DETECTED at {self.get_clock().now().to_msg()} "
                f"— max deviation {max_dev:.2f} m"
            )
            self.last_detection_time = self.get_clock().now()

        self.previous_path = current_pts

    def compute_deviation(self, old_pts, new_pts):
        """
        For every point in old_pts, find its nearest-neighbor distance to new_pts.
        Returns (max_deviation_value, list_of_bools_marking_points_above_point_threshold)
        """
        deviations = []
        for ox, oy in old_pts:
            nearest = min(math.hypot(ox - nx, oy - ny) for nx, ny in new_pts)
            deviations.append(nearest)

        moved_flags = [d > self.point_threshold for d in deviations]
        max_dev = max(deviations) if deviations else 0.0
        return max_dev, moved_flags

    def has_cluster(self, moved_flags):
        """
        Returns True if there's a run of at least self.cluster_size consecutive
        True values in moved_flags (i.e. a real obstacle-shaped detour, not scattered noise).
        """
        run_length = 0
        for moved in moved_flags:
            run_length = run_length + 1 if moved else 0
            if run_length >= self.cluster_size:
                return True
        return False

    def cooldown_elapsed(self):
        """
        True if we're allowed to log again — either never logged before,
        or enough time has passed since the last detection.
        """
        if self.last_detection_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return elapsed >= self.cooldown_sec


def main():
    rclpy.init()
    node = ReplanMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
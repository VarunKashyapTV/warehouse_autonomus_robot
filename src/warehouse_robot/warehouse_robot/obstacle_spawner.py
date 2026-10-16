import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
import math
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.subscription_1 = self.create_subscription(Path, "/plan", self.plan_callback, 10)
        self.subscription_2 = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        # TF, so map-frame plan points can be converted to the odom/Gazebo-world frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- tunable settings -------------------------------------------------
        self.spawn_distance = 0.9   # box appears when the robot is this close (m) to its spot
        self.min_goal_gap = 0.8     # never put a box closer than this (m) to the destination
        self.boxes_per_waypoint = 2 # the counter's start value: max boxes for each waypoint
        self.min_box_gap = 1.5      # a new box is at least this far (m) from every other box
        self.reroute_clearance = 0.3  # path points closer than this to a box = still going through it
        # a box is never planned closer than spawn_distance to the start of the route
        # ------------------------------------------------------------------------
        self.spawned_obstacle_names = []
        self.boxes_left = 0          # the counter: refilled by a new waypoint, -1 per spawn
        self.checkpoints_list = []   # the next box spot for the CURRENT waypoint (map frame)
        self.box_spots_map = []      # (x, y) of boxes already spawned for this waypoint (map frame)
        self.awaiting_reroute = False  # True after a spawn, until the planner routes around the box
        self.spawn_id = 0            # only ever increases; used purely for unique naming
        self.dest_x, self.dest_y = None, None
        self.dest_tolerance = 0.3    # plan end-points closer than this = same waypoint
        self.path_length = 0.0

    def arc_lengths(self, poses):
        arc = [0.0]
        for p, q in zip(poses, poses[1:]):
            arc.append(arc[-1] + math.hypot(p.pose.position.x - q.pose.position.x,
                                            p.pose.position.y - q.pose.position.y))
        return arc

    def pick_first_spot(self, poses):
        """Spot for the FIRST box of a new waypoint, on the original route:
        25% if two boxes are allowed, else (or if 25% is too close to the start) 50%.
        Never closer than spawn_distance to the start or min_goal_gap to the destination."""
        arc = self.arc_lengths(poses)
        total = arc[-1]
        for f in ([0.25, 0.5] if self.boxes_per_waypoint >= 2 else [0.5]):
            d = f * total
            if d < self.spawn_distance or total - d < self.min_goal_gap:
                continue
            idx = next(i for i, v in enumerate(arc) if v >= d)
            return [poses[idx]]
        return []

    def pick_next_spot(self, poses):
        """Spot for a LATER box, on the current (rerouted) path ahead of the robot.
        Tries the middle of what is left first, then slides along the path. Same gap rules
        as the first box, plus: at least min_box_gap from every box already spawned."""
        arc = self.arc_lengths(poses)
        total = arc[-1]
        for f in (0.5, 0.4, 0.6, 0.3, 0.7, 0.8):
            d = f * total
            if d < self.spawn_distance or total - d < self.min_goal_gap:
                continue
            idx = next(i for i, v in enumerate(arc) if v >= d)
            p = poses[idx].pose.position
            if all(math.hypot(p.x - bx, p.y - by) >= self.min_box_gap
                   for bx, by in self.box_spots_map):
                return [poses[idx]]
        return []

    def try_place_next_box(self, poses):
        """Called on every replan while waiting for the planner to route around the last box."""
        bx, by = self.box_spots_map[-1]
        if any(math.hypot(p.pose.position.x - bx, p.pose.position.y - by) < self.reroute_clearance
               for p in poses):
            return  # path still goes through the last box: planner has not rerouted yet
        self.awaiting_reroute = False
        self.checkpoints_list = self.pick_next_spot(poses)
        if self.checkpoints_list:
            self.get_logger().info("Rerouted: next obstacle planned on the new path")
        else:
            self.get_logger().info("Rerouted: no safe spot for another obstacle, skipping it")

    def plan_callback(self, msg: Path):
        if not msg.poses:  # empty path: nothing to read a destination from
            return
        end = msg.poses[-1].pose.position

        if (self.dest_x is not None
                and math.hypot(end.x - self.dest_x, end.y - self.dest_y) <= self.dest_tolerance):
            # same waypoint, just a replan. The only thing it can do is place the next
            # box once the planner has routed around the previous one.
            if self.awaiting_reroute:
                self.try_place_next_box(msg.poses)
            return

        # ---- a NEW waypoint ----
        self.dest_x, self.dest_y = end.x, end.y
        self.path_length = self.compute_path_length(msg.poses)
        self.clear_boxes()  # fresh obstacles for each waypoint
        self.boxes_left = self.boxes_per_waypoint  # refill the counter
        self.box_spots_map = []
        self.awaiting_reroute = False
        self.checkpoints_list = self.pick_first_spot(msg.poses)
        self.get_logger().info(
            f"New waypoint ({end.x:.2f}, {end.y:.2f}): route {self.path_length:.2f} m, "
            f"first obstacle {'planned' if self.checkpoints_list else 'not planned (route too short)'}, "
            f"counter = {self.boxes_left}")

    def clear_boxes(self):
        for name in self.spawned_obstacle_names:
            req = DeleteEntity.Request()
            req.name = name
            self.delete_cli.call_async(req).add_done_callback(self.del_response)
        self.spawned_obstacle_names.clear()

    def compute_path_length(self, poses):
        total = 0.0
        prev_x, prev_y = None, None  # None = "no point seen yet" (a real (0,0) is valid)
        for pose in poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            if prev_x is not None:
                total += math.hypot(prev_x - x, prev_y - y)
            prev_x, prev_y = x, y
        return total

    def map_to_odom(self, x, y):
        """Convert a point from the map frame to the odom frame (= Gazebo world frame).
        Returns (x, y), or None if the map->odom transform is not available yet."""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'map', Time())
        except TransformException as e:
            self.get_logger().warning(f"map->odom transform not available yet: {e}",
                                      throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * x - s * y + t.transform.translation.x,
                s * x + c * y + t.transform.translation.y)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.boxes_left <= 0 or not self.checkpoints_list:
            return  # counter at 0 (or nothing planned): no more spawns for this waypoint

        # Checkpoints come from /plan (map frame); the robot position comes from /odom
        # (odom frame). Bring the checkpoint into the odom frame before comparing.
        target = self.map_to_odom(self.checkpoints_list[0].pose.position.x,
                                  self.checkpoints_list[0].pose.position.y)
        if target is None:
            return
        tx, ty = target
        if math.hypot(self.current_x - tx, self.current_y - ty) <= self.spawn_distance:
            self.trigger_spawn(tx, ty)

    def trigger_spawn(self, x, y):
        self.spawn_obstacle(x, y)
        spot = self.checkpoints_list.pop(0)
        self.box_spots_map.append((spot.pose.position.x, spot.pose.position.y))
        self.boxes_left -= 1
        if self.boxes_left > 0:
            self.awaiting_reroute = True  # place the next box once the planner reroutes

    def spawn_obstacle(self, x, y):
        self.box_name = f"obstacle_box {self.spawn_id}"
        self.spawn_id += 1
        self.spawned_obstacle_names.append(self.box_name)
        self.sdf = f"""<sdf version="1.6">
                    <model name="{self.box_name}">
                    <static>true</static>
                    <link name="link">
                         <collision name="collision">
                         <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
                         </collision>
                         <visual name="visual">
                         <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
                         </visual>
                    </link>
                    </model>
                    </sdf>"""
        self.req = SpawnEntity.Request()
        self.req.name = self.box_name
        self.req.xml = self.sdf
        self.req.initial_pose.position.x = x
        self.req.initial_pose.position.y = y
        self.req.reference_frame = "map"

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.spawn_response)

    def del_response(self, future):
        self.get_logger().info(f"{future.result().success}")
        self.get_logger().info(f"{future.result().status_message}")

    def spawn_response(self, future):
        self.get_logger().info(f"{future.result().success}")
        self.get_logger().info(f"{future.result().status_message}")


def main():
    rclpy.init()
    node = ObstacleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Autonomous Warehouse Robot with Dynamic Path Replanning

A simulated warehouse robot built on ROS2 Nav2 that navigates autonomously between waypoints and replans its route in real time when a new obstacle appears mid-mission — without stopping or requiring human intervention.

## The Problem

In real warehouses, robots typically follow fixed, pre-planned paths. When something unexpected blocks the path mid-route — a worker walks in, a box falls, another robot crosses — many basic systems either stop and wait or fail outright. This project demonstrates a robot that detects a new obstacle and automatically finds a new route around it, continuing its mission without stopping.

## Tech Stack

| Tool | Role |
|---|---|
| ROS2 Humble | Core middleware connecting all nodes |
| Gazebo Classic 11 | Physics simulation |
| RViz2 | Visualization |
| Nav2 | Path planning, costmaps, and dynamic replanning |
| TurtleBot3 Burger | Simulated robot |
| SLAM Toolbox | Map building |
| AMCL | Localization against a known map |

## Environment

- WSL2, Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11

## Package Layout
warehouse_robot/
├── warehouse_robot/
│ ├── mission_controller.py — sequential waypoint navigation via Nav2 action client
│ ├── replan_monitor.py — detects real path replanning via nearest-neighbor deviation
│ └── obstacle_spawner.py — spawns/deletes boxes along the robot's path
├── launch/
│ └── warehouse_mission.launch.py
├── config/
│ └── nav2_params.yaml — project-specific Nav2 tuning (see below)
├── maps/
│ └── map.yaml / map.pgm
└── setup.py

## Running It

Two terminals are required. `mission_controller` is run separately from the main launch file, because multiple processes sharing one terminal under `ros2 launch` don't reliably route stdin to Python's `input()`.

**Terminal 1 — full stack:**
```bash
cd ~/ros2_ws
colcon build --packages-select warehouse_robot
source install/setup.bash
ros2 launch warehouse_robot warehouse_mission.launch.py
```

**Terminal 2 — once Gazebo, Nav2, and AMCL are up (wait for the initial pose to publish):**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run warehouse_robot mission_controller
```

At the prompt, enter waypoints as `x,y` (e.g. `1.2,1.3`), or `q` to quit. The controller sends one goal at a time and waits for it to complete before prompting for the next.

## How It Works

**Behavior 1 — Autonomous navigation.** `mission_controller` sends a `NavigateToPose` goal through Nav2's action client. Nav2 plans a path with the `NavFn` global planner (Dijkstra-based) and drives the robot there using the `DWB` local controller.

**Behavior 2 — Dynamic replanning.** `obstacle_spawner` watches the active `/plan` and the robot's `/odom` position, and spawns a box in Gazebo at a checkpoint along the current path once the robot gets close enough. The robot's laser scan picks up the new obstacle, Nav2's costmaps mark the area blocked, and the global planner recomputes a route around it. `replan_monitor` independently watches `/plan` for real replanning events (nearest-neighbor deviation between consecutive plans, filtered for a minimum consecutive-point cluster to avoid false positives from Nav2's routine path pruning) and logs them with a timestamp.

## Notable Fixes Along the Way

- **Threading race in `mission_controller`.** The terminal input loop and the Nav2 result callback run on different threads. A `threading.Event` gates the input loop so only one goal is ever in flight, preventing two `input()` calls from racing on the same stdin.
- **Costmap inflation too aggressive for a small map.** The map is only ~7.65m × 6.65m. Nav2's stock defaults (`inflation_radius: 0.55`, `cost_scaling_factor: 3.0`) inflated cost across nearly the whole map, sometimes including the robot's own start cell, which made the planner refuse to plan at all. Project-specific values (`inflation_radius: 0.3`, `cost_scaling_factor: 5.0`) live in `config/nav2_params.yaml`.
- **`/initialpose` published before the robot existed.** The original launch file published the initial pose on a fixed 25-second timer. Under WSL2 CPU contention, Gazebo's robot spawn sometimes took longer than that, so the pose was published into a world where the `odom` frame didn't exist yet and was silently dropped. The launch file now polls until `/odom` has a live publisher before publishing the initial pose.
- **AMCL recovery was disabled.** `recovery_alpha_fast`/`recovery_alpha_slow` were both `0.0`, so AMCL had no way to reseed its particle filter if it ever lost confidence — errors would only compound. Both are now enabled at conservative values.

## Known Open Items

- The DWB local controller sometimes hugs an obstacle's inflated edge too closely instead of committing to the wider route the global planner already found, occasionally triggering `Failed to make progress` aborts. Likely needs `BaseObstacle.scale` raised from its current very low value.
- `obstacle_spawner.py` has a couple of known-but-unfixed edge cases: a small pool for its obstacle-naming counter that can collide across long-running missions, and a `(0,0)` sentinel in its path-length calculation that can miscount a path passing near the robot's real starting coordinate.

## Demo

*(add a link to a recorded demo video here once available)*

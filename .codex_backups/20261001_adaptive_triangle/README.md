# Omnidirectional Robot Gazebo Simulation

## Introduction

This repository contains a Gazebo Ignition simulation that can be run with ROS 2, enabling the simulation of 3 to 6 wheeled omnidirectional robots. This setup is particularly useful for robotics research and simulation, allowing for the study and testing of complex robot behaviors and control strategies.

### Tested on

- Ubuntu 24.04
- ROS Jazzy
- Gazebo Harmonic

## Available Robot Model

![3W Robot Control](gif/3w.gif)  
*3 wheel omni robot (3w)*

![3W Robot Control v2](gif/3w_v2.gif)  
*3 wheel omni robot version 2 (3w_v2) -> default*

![4W Robot Control](gif/4w.gif)  
*4 wheel omni robot (4w)*

![5W Robot Control](gif/5w.gif)  
*5 wheel omni robot (5w)*

![6W Robot Control](gif/6w.gif)  
*6 wheel omni robot (6w)*

## How to Use This Repository

Please clone this repository to your ros2 workspace. The following assume your ros2 work space placed on your home directory with named **ros2_ws**

```bash
cd ~/ros2_ws/src
git clone git@github.com:YePeOn7/ros2_omni_robot_sim.git
```

Install dependency by running the following command

```bash
cd ~/ros2_ws/ros2_omni_robot_sim
sudo chmod +x install_dependency.sh
./install_dependency.sh
```

build the package by using the following command

```bash
colcon build
source install/setup.bash
```

Determine the robot Model by setting the environment variable of **OMNI_ROBOT_MODEL** with the robot model listed on [Available Robot Model](#available-robot-model). For example, if you want to use model of **3 wheel omni robot version 2**, you can set as follow:

```bash
export OMNI_ROBOT_MODEL=3w_v2
```

After that you can run several of the following simulations

### 1. Manual Teleop

```bash
ros2 launch ros2_omni_robot_sim gazebo_sim.launch.py
```

or you can specity the world as follow

```bash
ros2 launch ros2_omni_robot_sim gazebo_sim.launch.py world:=<world_name>
```

*please refer to [World Options](#world-options) Section for <world_name>*

![Robot Control](gif/control.gif)

### 2. SLAM with slam_toolbox

Start the SLAM by using the following command

```bash
ros2 launch ros2_omni_robot_sim slam_gazebo_sim.launch.py

# with specific world
ros2 launch ros2_omni_robot_sim slam_gazebo_sim.launch.py world:=<world_name>
```

*please refer to [World Options](#world-options) Section for <world_name>*

![Robot Control](gif/slam.gif)

Save the map by using the following command

```bash
cd ~/ros2_ws/ros2_omni_robot_sim/map # You can use other folder, however you need reconfigure the launch file to use your map
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

**Recommendation**: It is advised to use the **world name** as the **`<map_name>`** when saving your map. This ensures the map is automatically picked up by the launch file without additional configuration.

- For example: If your world is named **`maze1`**, name your map **`maze1`** as well.
- Saved files will be: `maze1.pgm` and `maze1.yaml`.

This consistency simplifies navigation setup and avoids manual path adjustments in your ROS 2 launch files.

### 3. Navigation

Start the SLAM by using the following command

```bash
ros2 launch ros2_omni_robot_sim navigation_gazebo_sim.launch.py

# with specific world
ros2 launch ros2_omni_robot_sim navigation_gazebo_sim.launch.py world:=<world_name>
```

*please refer to [World Options](#world-options) Section for <world_name>*

![Robot Control](gif/navigation.gif)

## Demo Video

[![Demo Video](https://img.youtube.com/vi/WKV7GRxiQOE/maxresdefault.jpg)](https://youtu.be/WKV7GRxiQOE)

## World Options

- maze1
- maze2 (default)

## Three-Robot D* Lite + Leader/Follower MPC Demo

Build the planner and simulator packages, source the workspace, then launch the
three `3w_v2` robots in `my_maze` with the map server and RViz:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select astar_planner ros2_omni_robot_sim
source install/setup.bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py
```

In RViz, select **2D Goal Pose** and click-drag a goal on the map. The node
publishes the leader's D* Lite route on `/dstar_lite/path`, robot paths on
`/robot1/desired_path` through `/robot3/desired_path`, and position/trail
markers on `/three_robot_dstar_mpc/markers`. Each robot runs its own
`robot_distributed_mpc` ROS node, consumes shared poses on
`/formation/robot_states`, exchanges horizon predictions on
`/robotN/mpc_prediction`, and publishes only to its `/robotN/cmd_vel` topic.
LiDAR returns add short-lived obstacles to the grid; changed `/map` cells are
applied incrementally by D* Lite.

To keep the map, planner, and spawn-position markers running without Gazebo or
robot motion, launch with:

```bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py simulation:=false motion_enabled:=false
```

### Coordinates and formation reference

The default simulation requires measured world positions from
`/world/my_maze/dynamic_pose/info`, bridged to `/gazebo/world_poses` as
`tf2_msgs/TFMessage`. The coordinator selects the three model names, publishes
their positions in `map` on `/formation/robot_states`, and supplies
`map -> robotN/odom` transforms. Link-relative entries from Gazebo are ignored.
The map raster uses the same origin and axes as the Gazebo world.

Before all three measured poses arrive, the markers show `spawn estimate` and
MPC commands stay zero. Missing/stale poses or references stop the controllers.
Odometry-only operation is an explicit coordinator option
`require_gazebo_pose:=false`; its conversion rotates displacement and yaw by the
spawn orientation before applying the spawn translation. This estimate can
still drift due to wheel slip, so the default simulation uses measured poses.

The default order is **robot3 (leader) -> robot2 -> robot1**, matching their
spawn positions. D* Lite plans for the leader. The followers track its measured
world-position trail with a target gap of **0.70 m along the trail**. The
initial trail connects the measured tail, middle and leader positions using
collision-checked D* routes around the third robot. Subsequent trail points
come from measured leader positions, not predicted future positions.

This version keeps a single-file formation in open areas as well as narrow
passages; lateral formation expansion is not enabled. Reference speed adapts
from 0.16 m/s to 0.08 m/s near walls and bends. Each follower's progress is
limited by its predecessor's measured progress. The leader slows or waits when
the followers fall behind, while the followers can continue catching up. The
RViz goal is the **leader's** destination; the other robots finish behind it
along the trail. The launch argument `leader_robot` changes the leader; the
other two are ordered by distance from its spawn position.

The inflated D* search grid uses a 0.18 m planning radius. Continuous pose and
segment checks use occupied cell boxes, avoiding unnecessary rejection caused
by rounding inflation to whole cells. A pose can join the search grid through
a short connector only when that entire connector has planning clearance.
MPC uses a 0.15 m circular collision radius and a soft cost that prefers 0.22 m
clearance. These values are specific to the current `3w_v2` geometry; the body
extends about 0.125 m from its center. Predicted robot centers must stay at
least 0.34 m apart.

If a measured pose has less than 0.18 m planning clearance but still meets the
0.15 m collision limit, the coordinator first looks for a short recovery path
away from the obstacle. Candidate paths are at most 0.40 m long, checked for
increasing clearance and stationary-peer separation. Recovery references move
one robot at a time at 0.05 m/s, then the convoy route is planned again. A pose
below the collision limit or with no valid recovery path is held and reported;
the code does not move it through a wall or teleport it onto the grid.

All three controllers consume **one shared reference message** on
`/formation/reference`. Each robot has its own progress, bounded by the convoy
spacing and measured tracking error. Each MPC uses a finite set of velocity
candidates over a receding horizon, with position/yaw tracking, control
smoothness, obstacle clearance and communicated peer predictions. The old
fixed relative XY formation cost is removed so it cannot pull followers across
the inside of a bend. Only that robot's MPC publishes its `cmd_vel`. The
coordinator, MPC nodes and RViz use `/clock` in simulation.

`/formation/reference` is a `PoseArray` in time-major order: robot1/robot2/robot3
at time zero, then the same three at each `mpc_dt` interval. Its length is
`3 * (mpc_horizon + 1)`. Each `/robotN/mpc_prediction` includes the current pose
followed by `mpc_horizon` future poses. Keep `mpc_dt` and `mpc_horizon` identical
on the coordinator and all three MPC nodes. The leader's `desired_path` shows
its planned route. Follower paths show the remaining measured leader trail
(plus initial connectors), extending to the last recorded leader position;
their current reference targets stop earlier to preserve the convoy gap.

After rebuilding, stop the previous launch with Ctrl+C before starting the new
one. On startup, wait for `robotN: using Gazebo world pose in map.` for all three
robots, then use **2D Goal Pose** in RViz. **2D Pose Estimate** does not teleport
Gazebo robots and is not used by this ground-truth simulation.

After selecting a free goal, expect `Leader/follower ready: robot3 -> robot2
-> robot1; gap 0.70 m`. If a robot is near a wall, `Clearance recovery` may
appear before that message. Start with a nearby goal ahead of robot3, then
choose a goal beyond a narrow passage. Observe the measured robot trails and
the spacing around corners in both Gazebo and RViz. Build success alone does
not establish that a complete passage traversal has been validated.

The pre-change planner, launch and README are backed up under
`.codex_backups/20260930_leader_follower/` at the workspace root. The original
A*/PID demo is separate from this launch.

Useful position diagnostics (Gazebo Fortress uses the `ign` command):

```bash
ros2 topic echo /formation/robot_states --once
ign topic -e -t /world/my_maze/dynamic_pose/info -n 1
```

`map/my_maze.pgm` is rasterized from static collision geometry in
`worlds/my_maze.sdf`; after editing world obstacles, regenerate it with:

```bash
python3 ros2_omni_robot_sim/scripts/generate_my_maze_map.py
```

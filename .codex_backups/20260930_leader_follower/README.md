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

## Three-Robot D* Lite + MPC Formation Demo

Build the planner and simulator packages, source the workspace, then launch the
three `3w_v2` robots in `my_maze` with the map server and RViz:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select astar_planner ros2_omni_robot_sim
source install/setup.bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py
```

In RViz, select **2D Goal Pose** and click-drag a goal on the map. The node
publishes the D* Lite route on `/dstar_lite/path`, adaptive paths on
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

Robot2 anchors the D* Lite route. The other robots change from lateral slots
into a convoy with a configured gap of 0.65 m. The adaptive structure searches
width transitions along the route, checking wall clearance and the minimum
distance between robots throughout each movement. If necessary, initial
assembly moves one robot at a time along D* routes around stationary teammates.
The inflated grid remains conservative for D* search. At measured positions,
the planner and MPC check the swept circular footprint against occupied cell
boxes using the same 0.18 m clearance. This avoids rejecting a pose simply
because rounding the inflation up to a whole cell marked its cell occupied.
Such a pose joins the search grid through a short, collision-checked connector;
the connector is included in the robot's path. Clearance is not reduced.

If formation assembly cannot start at the first center waypoint, the planner
also tries joining a viable suffix of the route farther ahead. Robots assemble
one at a time along checked D* routes while the other robots stay in place.
Swept obstacle checks and inter-robot separation apply during these movements.
The planner reports whether rejection occurred at the goal, at measured robot
clearance, or while attempting initial assembly. Goals without footprint
clearance or a safe connector to the search grid are rejected.

All three controllers consume **one shared timed reference** on
`/formation/reference`; they do not advance independently along their displayed
paths. The shared phase slows or holds when any robot falls behind. Each MPC
uses a finite set of velocity candidates over a receding horizon, including
tracking, relative formation error, control smoothness, obstacle constraints,
and communicated peer predictions. Only that robot's MPC publishes its
`cmd_vel`. The coordinator, MPC nodes and RViz use `/clock` in simulation.

`/formation/reference` is a `PoseArray` in time-major order: robot1/robot2/robot3
at time zero, then the same three at each `mpc_dt` interval. Its length is
`3 * (mpc_horizon + 1)`. Each `/robotN/mpc_prediction` includes the current pose
followed by `mpc_horizon` future poses. Keep `mpc_dt` and `mpc_horizon` identical
on the coordinator and all three MPC nodes. The displayed `desired_path` topics
are the geometric paths of these same formation references.

After rebuilding, stop the previous launch with Ctrl+C before starting the new
one. On startup, wait for `robotN: using Gazebo world pose in map.` for all three
robots, then use **2D Goal Pose** in RViz. **2D Pose Estimate** does not teleport
Gazebo robots and is not used by this ground-truth simulation.

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

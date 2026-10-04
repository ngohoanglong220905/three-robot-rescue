# Three Robot Rescue

Mô phỏng 3 robot omni `3w_v2` trên world `my_maze` bằng ROS 2 Humble và Gazebo.

## Các package

- `astar_planner`: D* Lite, MPC và điều khiển đội hình thích nghi.
- `ros2_omni_robot_sim`: model robot, world, map, launch và cấu hình RViz.
- `multi_robot_formation`: các node điều khiển đội hình.
- `virtual_structure_3robot`: triển khai virtual structure.

## Build và chạy

Clone repository vào workspace, sau đó chạy tại thư mục repository:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select astar_planner ros2_omni_robot_sim
source install/setup.bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py
```

Xem hướng dẫn cấu hình, phụ thuộc và xử lý đồ họa trong [README mô phỏng](ros2_omni_robot_sim/README.md) và [README MPC và đội hình](ros2_omni_robot_sim/README_MPC_FORMATION.md).

Giữ các thông tin tác giả và giấy phép có sẵn trong từng package.

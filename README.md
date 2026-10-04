# ROS 2 Workspace — Three Robot Rescue

Mô phỏng 3 robot omni `3w_v2` trên world `my_maze` bằng ROS 2 Humble và Gazebo.

## Phạm vi repository

Repository chứa bản chụp toàn bộ workspace `~/ros2_ws`, bao gồm 4 package nguồn, `build/`, `install/`, `log/`, `.codex_backups/`, `.vscode/` và các cache hiện có. Metadata `.git` của repository gốc trong package không được sao chép; đây là repository chung cho workspace.

Các file build/install có liên kết tuyệt đối tới `/home/long/ros2_ws` và phụ thuộc môi trường máy đã build. Khi clone sang thư mục hoặc máy khác, cần xóa các thư mục tự sinh trong **bản clone** và build lại từ nguồn.

## Các package

- `astar_planner`: D* Lite, MPC và điều khiển đội hình thích nghi.
- `ros2_omni_robot_sim`: model robot, world, map, launch và cấu hình RViz.
- `multi_robot_formation`: các node điều khiển đội hình.
- `virtual_structure_3robot`: triển khai virtual structure.

## Build và chạy

Clone repository vào workspace, sau đó chạy tại thư mục repository:

```bash
# Chỉ chạy tại thư mục bản clone mới; xóa output build đi kèm snapshot.
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select astar_planner ros2_omni_robot_sim
source install/setup.bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py
```

Xem hướng dẫn cấu hình, phụ thuộc và xử lý đồ họa trong [README mô phỏng](ros2_omni_robot_sim/README.md) và [README MPC và đội hình](ros2_omni_robot_sim/README_MPC_FORMATION.md).

Giữ các thông tin tác giả và giấy phép có sẵn trong từng package.

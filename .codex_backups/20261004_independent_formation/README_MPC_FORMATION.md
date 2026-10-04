# Ba robot 3w_v2: D* Lite, đội hình thích nghi và MPC

Hướng dẫn này dành cho demo ba robot `3w_v2` trong world `my_maze`. Mặc định `robot3` là leader; `robot2` và `robot1` đi theo. Đích đặt trong RViz là **đích của leader**.

## Các file cần biết

Đường dẫn trong bảng tính từ thư mục workspace `~/ros2_ws`.

| File | Vai trò |
| --- | --- |
| `ros2_omni_robot_sim/launch/three_robot_dstar_mpc.launch.py` | Điểm chạy chính: khởi động Gazebo, map server, RViz, một node điều phối D* Lite và ba node MPC. Đặt vị trí xuất phát, leader và tham số đội hình. |
| `ros2_omni_robot_sim/launch/three_robot_gazebo.launch.py` | Nạp world `my_maze`, sinh ba robot và kết nối mô phỏng với ROS 2. Launch chính truyền vị trí xuất phát riêng cho demo này. |
| `ros2_omni_robot_sim/launch/_graphics_backend.py` | Chọn môi trường OpenGL/Mesa dùng chung cho RViz và Gazebo; phát hiện NVIDIA lệch phiên bản và chọn GPU Intel hoặc CPU. |
| `astar_planner/astar_planner/three_robot_dstar_mpc.py` | Chứa `DStarLiteGrid` để tìm đường và cập nhật đường khi map đổi; `ThreeRobotDStarMPC` tạo đường, đội hình và quỹ đạo tham chiếu; `DistributedRobotMPC` tính lệnh vận tốc cho **một** robot từ tham chiếu, map, LiDAR và dự đoán của các robot còn lại. |
| `astar_planner/astar_planner/adaptive_leader.py` | Chọn giữa tam giác mặc định và hàng dọc theo không gian trống; phát hiện khe hẹp phía trước, kiểm tra chỗ đứng và đường chuyển đội hình. |
| `astar_planner/astar_planner/leader_follower.py` | Lưu vệt vị trí đã đo của leader và đặt mục tiêu cho follower bám theo vệt đó khi đi hàng dọc. |
| `astar_planner/astar_planner/formation_motion.py` | Kiểm tra chuyển động của cả nhóm và chia quá trình ghép/tách đội hình thành từng giai đoạn. |
| `astar_planner/astar_planner/adaptive_structure.py` | Cung cấp phép nội suy và kiểm tra khoảng cách trên cả đoạn chuyển động; một số phần của điều phối và MPC dùng các hàm hình học này. |
| `astar_planner/astar_planner/turnaround.py` | `RollingTurnMotion` cho leader quay theo đường rồi đi ngay khi đủ hướng; follower điều chỉnh vị trí đồng thời theo đường riêng có kiểm tra va chạm. Chỉ dùng các bước nhường chỗ riêng khi không có cách đi đồng thời an toàn. |
| `astar_planner/astar_planner/grid_clearance.py`, `peer_safety.py`, `separation_recovery.py` | Kiểm tra khoảng cách tới tường và giữa các robot; tìm chuyển động phục hồi khi hai robot ở quá gần. |
| `ros2_omni_robot_sim/worlds/my_maze.sdf` | Hình học mê cung và vật cản trong Gazebo. |
| `ros2_omni_robot_sim/map/my_maze.yaml`, `my_maze.pgm` | Occupancy grid mà D* Lite và MPC sử dụng; tọa độ map khớp world Gazebo. |
| `ros2_omni_robot_sim/config/gz_bridge/gz_bridge_3robots.yaml` | Chuyển `/clock`, vị trí world và dữ liệu cảm biến từ Gazebo sang ROS 2; vị trí ba robot đến node điều phối qua `/gazebo/world_poses`. |
| `ros2_omni_robot_sim/rviz/three_robot_dstar_mpc.rviz` | Cấu hình RViz để xem map, robot, đường đi và đội hình. |
| `astar_planner/setup.py` | Đăng ký hai executable ROS 2: `three_robot_dstar_mpc` và `robot_distributed_mpc`. |

`astar_node.py` và `pid_follower.py` là các demo khác, không phải node điều khiển của launch này. Chỉ sửa file trong thư mục nguồn `astar_planner/` hoặc `ros2_omni_robot_sim/`; các bản trong `build/` và `install/` được tạo lại khi build.

## Đường đi và đội hình hoạt động thế nào?

1. RViz gửi `/goal_pose`. Node điều phối nhận `/map` và vị trí **đã đo** của ba robot từ Gazebo, rồi dùng D* Lite tìm đường cho leader. Đường toàn cục được phát trên `/dstar_lite/path`.
2. Ở vùng rộng, `robot3` dẫn đầu và `robot2`/`robot1` đứng hai bên phía sau thành tam giác. Khi phát hiện đường phía trước không đủ bề ngang, hệ thống bắt đầu chuyển **thẳng sang hàng dọc**, bỏ các mức thu nhỏ tam giác trung gian. Thứ tự là `robot3 → robot2 → robot1`, khoảng cách bám theo vệt là 0,70 m. Hệ thống giữ hàng dọc đến khi cả đội và đường phía trước đủ khoảng trống liên tục ít nhất 0,60 s, rồi mới mở lại tam giác.
3. Node điều phối phát vị trí chung trên `/formation/robot_states` và quỹ đạo tham chiếu chung trên `/formation/reference`. Mỗi node `robot_distributed_mpc` dự đoán chuyển động trong một khoảng thời gian ngắn, tránh tường và robot khác, rồi **chỉ** phát lệnh cho `/robot1/cmd_vel`, `/robot2/cmd_vel` hoặc `/robot3/cmd_vel` tương ứng. Bước kế tiếp được tính lại theo vị trí mới đo được.
4. Khi quay đầu hoặc đổi hướng lớn, leader quay theo hướng đoạn đường tiếp theo. Khi sai lệch hướng đo được còn tối đa **0,20 rad (khoảng 11°)**, leader được đi ngay nếu đường trống; hai follower vẫn có thể đang điều chỉnh vị trí. Mỗi robot có tiến độ riêng, và các đoạn chuyển động được kiểm tra lại theo vị trí thực tế. Nếu follower đang chắn đường và không có cách di chuyển đồng thời an toàn, hệ thống mới dùng các bước nhường chỗ riêng.

Mặc định, tam giác có bề ngang 1,00 m; hai follower ở sau leader 0,70 m. Vị trí xuất phát lần lượt là `robot1=(4.0, -2.9)`, `robot2=(3.0, -2.9)`, `robot3=(3.5, -2.2)` trong frame `map`.

Việc thu về hàng dọc được kiểm tra mỗi chu kỳ điều khiển 0,10 s và có thể hủy một lần mở tam giác đang diễn ra nếu phát hiện khe hẹp mới. Tốc độ tham chiếu khi thu/mở đội hình mặc định là **0,18 m/s**; tốc độ đi trong khe hẹp vẫn là 0,08 m/s. Hai follower di chuyển đồng thời nếu các đường đi và khoảng cách giữa robot cho phép; nếu không, chúng chuyển lần lượt. Leader giữ vị trí trong lúc chuyển và chỉ tiếp tục khi vị trí đo được đã đạt đội hình mới.

Trong lúc đổi hướng, đội hình được phép giãn tạm thời để follower nhập lại phía sau leader khi an toàn; không bắt cả đội quay cứng quanh một tâm. Khi leader đã qua đoạn quay và follower nhập lại, hệ thống tiếp tục chọn tam giác hoặc hàng dọc theo không gian trống. Trong terminal, các dòng `turn / aligning leader; followers tracking` và `turn / leader moving; followers tracking` cho biết trạng thái này. Các dòng `turnaround / followers yield` chỉ xuất hiện khi cần nhường chỗ riêng.

Kiểm tra MPC bằng mô hình động học trên `my_maze` cho thấy leader bắt đầu đi sau 3,5 s khi quay 180°; cách chia giai đoạn trước đó mất 8,4 s trong ca đối chiếu ở vùng trống. Đây là thời gian trong kiểm tra động học, chưa phải kết quả đo với bánh xe và tiếp xúc vật lý của Gazebo. Bản mã trước thay đổi quay linh hoạt được lưu tại `~/ros2_ws/.codex_backups/20261004_rolling_turn/`.

## Chạy trong terminal

Mở **terminal 1** và chạy từ workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select astar_planner ros2_omni_robot_sim
source install/setup.bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py
```

Đợi Gazebo tải đủ ba robot và RViz hiển thị map. Trong RViz, chọn **2D Goal Pose**, nhấp và kéo tới một ô trống trên map để đặt đích cho leader. Dùng `Ctrl+C` trong terminal 1 để dừng; dừng bản chạy cũ trước khi launch lại để tránh các node trùng tên.

Mở **terminal 2** nếu muốn theo dõi các topic (sau khi đã đặt goal):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic echo --once /formation/robot_states
ros2 topic echo --once /dstar_lite/path
ros2 topic echo --once /robot3/cmd_vel
```

Các lệnh `echo --once` chờ đến khi có thông điệp mới; `/dstar_lite/path` cần một goal hợp lệ và có thể cần đặt lại goal nếu lệnh được chạy sau khi đường đã phát.

Các chế độ launch hữu ích:

```bash
# Có Gazebo và RViz, nhưng ba robot không nhận lệnh di chuyển.
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py motion_enabled:=false

# Chỉ xem map, planner và RViz; không mở Gazebo, không cho robot di chuyển.
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py simulation:=false motion_enabled:=false

# Chỉ dùng đội hình hàng dọc.
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py adaptive_formation:=false
```

Có thể điều chỉnh tốc độ thu/mở và thời gian xác nhận vùng rộng ngay khi launch, ví dụ:

```bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py formation_transition_speed:=0.18 formation_open_hold:=0.60
```

`formation_transition_speed` là tốc độ tham chiếu (m/s), không vượt `max_linear_speed` của node điều phối, mặc định 0,28 m/s. `formation_open_hold` tính bằng giây và chỉ áp dụng khi mở lại tam giác; phát hiện khe hẹp không phải chờ khoảng thời gian này. MPC vẫn có thể giảm tốc hoặc dừng nếu khoảng cách thực tế không cho phép.

Chạy từng chế độ riêng sau khi dừng phiên trước. Nếu thay đổi các file Python hoặc launch, chạy lại lệnh `colcon build` rồi `source install/setup.bash` trong terminal dùng để launch.

## Khi RViz hoặc Gazebo báo lỗi OpenGL

Lỗi `Unable to create a suitable GLXContext` xảy ra khi RViz không tạo được ngữ cảnh OpenGL. Trên máy hiện tại, `nvidia-smi` báo `Driver/library version mismatch`: module NVIDIA đang nạp là `580.173.02`, còn thư viện là `580.178`. RViz mở được với GPU Intel qua Mesa (OpenGL 4.6) và với chế độ CPU (OpenGL 4.5).

Gazebo có thể gặp cùng lỗi: trong `~/.ignition/auto_default.log` xuất hiện `Failed to create OpenGL context`, GUI thoát và lệnh Gazebo tổng gửi SIGINT cho server. Vì server đã dừng, các tiến trình `create` không thể sinh robot. Việc terminal báo Gazebo `finished cleanly` không có nghĩa world vẫn đang chạy.

Launch có hai tham số **độc lập**, `rviz_rendering` và `gazebo_rendering`, đều mặc định là `auto`. `gazebo_rendering` áp dụng cho cả GUI và renderer cảm biến của server. Các giá trị giống nhau:

| Giá trị | Cách chọn renderer |
| --- | --- |
| `auto` | Dùng cấu hình đồ họa hiện tại. Nếu `nvidia-smi` báo lệch phiên bản driver/thư viện, chuyển sang GPU Intel nếu tìm được thiết bị render Intel; nếu không, dùng CPU. |
| `intel` | Dùng Mesa trên GPU Intel; tự tìm địa chỉ PCI của GPU. Nếu máy không có thiết bị render Intel, launch báo rõ lỗi. |
| `software` | Dùng Mesa render trên CPU; có thể giảm tốc độ hiển thị và tăng tải CPU. |
| `native` | Dùng môi trường đồ họa của terminal, không ghi đè renderer. |

Để chọn GPU Intel cho cả hai cửa sổ ngay trên máy này, dừng phiên launch trước bằng `Ctrl+C` rồi chạy:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros2_omni_robot_sim
source install/setup.bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py gazebo_rendering:=intel rviz_rendering:=intel
```

Nếu vẫn không tạo được GLXContext, chọn CPU:

```bash
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py gazebo_rendering:=software rviz_rendering:=software
```

Các biến môi trường đồ họa được giới hạn trong tiến trình RViz và nhóm launch Gazebo. `DRI_PRIME` chọn GPU Intel theo địa chỉ PCI, và `LIBGL_ALWAYS_SOFTWARE=true` chọn render bằng CPU theo [tài liệu Mesa](https://docs.mesa3d.org/envvars.html). Nếu có cấu hình EGL Mesa, renderer cảm biến cũng được chỉ định dùng thư viện EGL Mesa. Chế độ này cần phiên desktop X11/XWayland với `DISPLAY` hợp lệ.

Nếu Gazebo thoát, launch mới dừng các node còn lại thay vì để tiến trình sinh robot chờ mãi. Mỗi lệnh `create` cũng chỉ định trực tiếp world `my_maze` bằng `-world my_maze`; ba robot vẫn được sinh lần lượt.

Kiểm tra khởi động ngày 04/10/2026 với cả hai renderer để `auto` và `motion_enabled:=false`: launch chọn Intel, sinh đủ `robot1`, `robot2`, `robot3`, nhận vị trí world của cả ba, `/clock` và ba topic LiDAR. Cả ba bộ controller bánh xe được kích hoạt; Gazebo vẫn chạy đến khi chủ động dừng phiên kiểm tra. Đây là kiểm tra khởi động, không phải kiểm tra di chuyển đội hình.

Module NVIDIA trên ổ đĩa đã là `580.178.04` (`modinfo -F version nvidia`), nhưng hệ thống vẫn đang dùng module cũ. Để xử lý lệch phiên bản này, lưu công việc rồi **khởi động lại máy** để nạp module mới. Sau khi khởi động lại, kiểm tra:

```bash
nvidia-smi
cat /proc/driver/nvidia/version
```

Nếu `nvidia-smi` đã hiển thị GPU bình thường, có thể dùng lại chế độ mặc định `auto` hoặc chỉ định `gazebo_rendering:=native rviz_rendering:=native`. Nếu vẫn báo lệch phiên bản, cần sửa bộ cài driver NVIDIA của hệ thống; tùy chọn renderer trong project chỉ giúp chạy giao diện và cảm biến bằng GPU khác/CPU, không sửa driver hệ thống.

## Khi robot không đi

- Kiểm tra Gazebo đã sinh đủ ba robot và `/formation/robot_states` có vị trí đo được. Trong chế độ mô phỏng mặc định, MPC giữ lệnh bằng 0 nếu thiếu vị trí hoặc tham chiếu mới.
- Đặt goal trên ô trống, tránh tường và vật cản. Cảnh báo D* Lite không tìm được đường thường cho biết điểm đầu/đích hoặc lối nối vào lưới không đủ khoảng trống an toàn.
- Khi follower quá gần nhau hoặc đang sát tường, chờ bước phục hồi khoảng cách; node có thể giữ leader đứng yên trong lúc các robot còn lại đổi vị trí.

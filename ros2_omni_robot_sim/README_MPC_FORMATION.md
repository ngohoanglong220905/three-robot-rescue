# Ba robot 3w_v2: D* Lite, đội hình thích nghi và MPC

**Mới:** launch xã hội mặc định dùng `social_people`, sàn mở có nhóm trò chuyện,
hai người ngồi ghế xem TV, pose cơ thể và lịch sử 10 s. Vùng tránh tạm được
tạo từ cảm biến trong lúc chờ xác nhận. Xem [hướng dẫn thử vùng xã hội](README_SOCIAL_PEOPLE.md).
Map phòng/TV bên dưới chọn bằng `social_scene:=rooms`.

Hướng dẫn này dành cho demo ba robot `3w_v2` trong world `my_maze`. Mặc định `robot3` là leader; `robot2` và `robot1` đi theo. Đích đặt trong RViz là **đích của leader**.

World **`social_rescue` đã thu gọn còn 12 × 10 m**, có 4 phòng, hành lang/ngõ rẽ,
10 người, gồm 3 người mới dọc hành lang và một người cách leader khoảng 2,4 m
ở phía trước lúc xuất phát; vẫn có nhóm trò chuyện, xem TV và vật cản thấp/cao. Launch
tự bật YOLOv4-tiny nhận diện người từ camera RGB-D của leader. Chạy bằng
`ros2 launch ros2_omni_robot_sim three_robot_social.launch.py social_scene:=rooms`.
Xem [hướng dẫn map xã hội](README_SOCIAL_SCENE.md) để biết bố trí và các file.
Kết quả camera nằm ở `/leader/people/...`; vùng chú thích kịch bản ở
`/social_scene/...`. Launch xã hội đã bổ sung hướng đầu ước lượng, vùng tránh
giữa nhóm người/TV và lượt quay quan sát qua MPC; xem hướng dẫn map xã hội
để gọi service quét 360° và xem giới hạn của suy luận từ ảnh.

## Các file cần biết

Đường dẫn trong bảng tính từ thư mục workspace `~/ros2_ws`.

| File | Vai trò |
| --- | --- |
| `ros2_omni_robot_sim/launch/three_robot_dstar_mpc.launch.py` | Điểm chạy chính: khởi động Gazebo, map server, RViz, một node điều phối D* Lite và ba node MPC. Đặt vị trí xuất phát, leader và tham số đội hình. |
| `ros2_omni_robot_sim/launch/three_robot_gazebo.launch.py` | Nạp world `my_maze`, sinh ba robot và kết nối mô phỏng với ROS 2. Launch chính truyền vị trí xuất phát riêng cho demo này. |
| `ros2_omni_robot_sim/launch/_graphics_backend.py` | Chọn môi trường OpenGL/Mesa dùng chung cho RViz và Gazebo; phát hiện NVIDIA lệch phiên bản và chọn GPU Intel hoặc CPU. |
| `astar_planner/astar_planner/three_robot_dstar_mpc.py` | Chứa `DStarLiteGrid` để tìm đường và cập nhật đường khi map đổi; `ThreeRobotDStarMPC` tạo đường, đội hình và quỹ đạo tham chiếu; `DistributedRobotMPC` tính lệnh vận tốc cho **một** robot từ tham chiếu, map, LiDAR và dự đoán của các robot còn lại. |
| `astar_planner/astar_planner/moving_formation.py` | `MovingAdaptiveFormation` chọn tam giác/hàng dọc, phối hợp tốc độ theo khoảng cách đo được giữa các robot và cố định slot cuối đường. `FollowerTrack` lưu đường nhập đội hình; `checked_advance` đi qua nhiều điểm vệt nhỏ khi đoạn nối an toàn; `checked_team_row` kiểm tra chuyển động đồng thời của cả nhóm. |
| `astar_planner/astar_planner/adaptive_leader.py` | Cung cấp hình học virtual structure, kiểm tra không gian cho tam giác và phần triển khai chuyển đội hình đứng yên trước đây. `MovingAdaptiveFormation` kế thừa phần hình học này và thay cách chuyển/điều phối. |
| `astar_planner/astar_planner/leader_follower.py` | Lưu vệt vị trí đã đo của leader và đặt mục tiêu cho follower bám theo vệt đó khi đi hàng dọc. |
| `astar_planner/astar_planner/formation_motion.py` | Kiểm tra chuyển động của cả nhóm và chia quá trình ghép/tách đội hình thành từng giai đoạn. |
| `astar_planner/astar_planner/adaptive_structure.py` | Cung cấp phép nội suy và kiểm tra khoảng cách trên cả đoạn chuyển động; một số phần của điều phối và MPC dùng các hàm hình học này. |
| `astar_planner/astar_planner/turnaround.py` | `leader_route_blockers` kiểm tra follower nào chắn đường leader trong 1,20 m phía trước. `yield_followers` chỉ tìm chỗ né cho robot đang chắn, ưu tiên quãng dịch chuyển ngắn; `RollingTurnMotion` thực hiện đường nhường chỗ và cho leader đi khi khoảng cách đo được cho phép. Các hàm ghép cả đội và quay theo giai đoạn trước đây được giữ lại để tham khảo. |
| `astar_planner/astar_planner/grid_clearance.py`, `peer_safety.py`, `separation_recovery.py` | Kiểm tra khoảng cách tới tường và giữa các robot; tìm chuyển động phục hồi khi hai robot ở quá gần. |
| `ros2_omni_robot_sim/worlds/my_maze.sdf` | Hình học mê cung và vật cản trong Gazebo. |
| `ros2_omni_robot_sim/map/my_maze.yaml`, `my_maze.pgm` | Occupancy grid mà D* Lite và MPC sử dụng; tọa độ map khớp world Gazebo. |
| `ros2_omni_robot_sim/config/gz_bridge/gz_bridge_3robots.yaml` | Chuyển `/clock`, vị trí world và dữ liệu cảm biến từ Gazebo sang ROS 2; vị trí ba robot đến node điều phối qua `/gazebo/world_poses`. |
| `ros2_omni_robot_sim/rviz/three_robot_dstar_mpc.rviz` | Cấu hình RViz để xem map, robot, đường đi và đội hình. |
| `ros2_omni_robot_sim/urdf/3w_v2_multi/leader_camera.urdf.xacro` | Cụm RGB-D của leader: giá đỡ, vỏ cam–đen, mắt kính, sensor và frame quang học. |
| `ros2_omni_robot_sim/config/gz_bridge/leader_camera_bridge.yaml` | Bridge riêng cho ảnh màu, ảnh chiều sâu và nội tham số camera; chỉ truyền topic đang có subscriber. |
| `ros2_omni_robot_sim/scripts/leader_obstacle_scan.py` | Lọc mặt sàn/thân xe từ point cloud LiDAR nhiều tầng của leader rồi tạo LaserScan cho D* Lite và MPC. |
| `ros2_omni_robot_sim/urdf/3w_v2_multi/base_link.urdf.xacro`, `main.urdf.xacro` | Chọn cụm camera cho leader và nâng LiDAR lên trên giá camera; follower giữ cấu hình cảm biến cũ. |
| `astar_planner/setup.py` | Đăng ký hai executable ROS 2: `three_robot_dstar_mpc` và `robot_distributed_mpc`. |

`astar_node.py` và `pid_follower.py` là các demo khác, không phải node điều khiển của launch này. Chỉ sửa file trong thư mục nguồn `astar_planner/` hoặc `ros2_omni_robot_sim/`; các bản trong `build/` và `install/` được tạo lại khi build.

## Đường đi và đội hình hoạt động thế nào?

1. RViz gửi `/goal_pose`. Node điều phối nhận `/map` và vị trí **đã đo** của ba robot từ Gazebo, rồi dùng D* Lite tìm đường cho leader. Đường toàn cục được phát trên `/dstar_lite/path`.
2. Ở vùng rộng, `robot3` dẫn đầu và `robot2`/`robot1` đứng hai bên phía sau thành tam giác. Khi phát hiện đường phía trước không đủ bề ngang, hệ thống bắt đầu chuyển **thẳng sang hàng dọc**, bỏ các mức thu nhỏ tam giác trung gian. Thứ tự xuất phát là `robot3 → robot2 → robot1`, khoảng cách bám theo vệt là 0,70 m. Sau khi leader đi qua chỗ nhường, nếu follower cuối gần leader hơn follower giữa ít nhất 0,15 m thì đổi thứ tự hai follower để giảm quãng nhập lại đội hình; leader vẫn là `robot3`. Hệ thống giữ hàng dọc đến khi cả đội và đường phía trước đủ khoảng trống liên tục ít nhất 0,60 s, rồi mới mở lại tam giác.
3. Node điều phối phát vị trí chung trên `/formation/robot_states` và quỹ đạo tham chiếu chung trên `/formation/reference`. Mỗi node `robot_distributed_mpc` dự đoán chuyển động trong một khoảng thời gian ngắn, tránh tường và robot khác, rồi **chỉ** phát lệnh cho `/robot1/cmd_vel`, `/robot2/cmd_vel` hoặc `/robot3/cmd_vel` tương ứng. Bước kế tiếp được tính lại theo vị trí mới đo được.
4. Khi đổi hướng hoặc quay đầu, trước hết kiểm tra **đường leader có bị follower chắn thật sự hay không**, không tạo bước quay riêng chỉ vì góc đổi hướng lớn. Nếu không bị chắn, leader tiếp tục bám D* và điều chỉnh yaw đồng thời; robot omni có thể dịch chuyển và quay cùng lúc. Nếu bị chắn, chỉ follower liên quan được định tuyến đến chỗ né gần nhất có đủ khoảng trống. Follower còn lại vẫn được tạo mục tiêu bám phía sau tiến độ leader đã đo nếu có đường nhập an toàn. Leader có thể tiếp tục tiến tới phần đường đang an toàn trong lúc follower né và đi qua ngay khi khoảng cách đo được cho phép; không chờ cả đội ghép xong đội hình.

Mặc định, tam giác có bề ngang 1,00 m; hai follower ở sau leader 0,70 m. Vị trí xuất phát lần lượt là `robot1=(4.0, -2.9)`, `robot2=(3.0, -2.9)`, `robot3=(3.5, -2.2)` trong frame `map`.

Việc thu về hàng dọc được kiểm tra mỗi chu kỳ điều khiển 0,10 s. Khi phát hiện khe hẹp, **mục tiêu đội hình đổi sang hàng dọc ngay trong chu kỳ đó**, rồi hai follower nhập vào vệt leader bằng các đoạn đường đã kiểm tra. Chuyển động vật lý vẫn chịu giới hạn vận tốc/gia tốc; ba robot không thể đổi vị trí tức thời. Leader tiếp tục đi nếu đường của nó an toàn, thay vì đứng chờ follower đạt đúng slot.

Tốc độ tham chiếu leader ở vùng rộng là 0,16 m/s; follower bám/nhập đội hình mặc định 0,22 m/s. Trong hàng dọc qua khe hẹp, leader dùng tối đa 0,08 m/s; follower được dùng tới **0,12 m/s khi cần bắt kịp**, nhưng mục tiêu vẫn bị giới hạn bởi vị trí đo được của robot phía trước. MPC vẫn giới hạn vận tốc tối đa 0,28 m/s, gia tốc và khoảng cách thực tế; các giá trị trên là tốc độ tham chiếu, không phải vận tốc thực tế được bảo đảm.

### Bám đuôi qua khe hẹp

- Hai follower dùng **cùng tiến độ leader** để tạo các slot cách nhau 0,70 m dọc theo vệt đã đo. Cả hai nhận quỹ đạo theo thời gian trên vệt này; follower cuối không phải đợi follower giữa đến đúng slot rồi mới nhận phần đường tiếp theo. Các mục tiêu vẫn phải nằm phía sau vị trí đã đo của robot trước, và các đoạn chuyển động đều được kiểm tra va chạm.
- Các điểm vệt cách nhau khoảng 1,5 cm được đi qua liên tục nếu đoạn nối an toàn. Khi đoạn nối có thể cắt vào tường, bộ tạo tham chiếu giữ lại góc đường. Hướng follower lấy theo tiếp tuyến làm trơn trên vệt, giúp giảm việc quay theo nhiễu của từng điểm nhỏ.
- Nếu một khoảng cách dọc vượt mức đặt hơn 0,08 m, leader giảm tốc theo độ giãn. Robot giữa cũng giảm tốc khi robot cuối tụt lại. Khi độ giãn hoặc sai số nhập vệt đạt 0,38 m, robot phía trước có thể tạm dừng để thu đội hình; kiểm tra tránh va chạm vẫn có quyền giữ từng robot.
- Leader giữ tốc độ khe hẹp khi còn follower ở chỗ hẹp. Chế độ này bật khi khoảng trống tới tường dưới 0,32 m, và chỉ bỏ khi cả nhóm cùng đoạn vệt phía trước đạt ít nhất 0,36 m, tránh tăng/giảm tốc liên tục tại cửa khe.
- Khi leader còn dưới 0,05 m tới đích và đoạn cuối an toàn, các slot kết thúc hàng dọc được cố định theo cuối vệt. Hai follower cùng hội tụ về slot của mình, cách nhau 0,70 m; **không chạy cả ba robot vào cùng tọa độ goal**.

Khi cả nhóm và đường phía trước đủ rộng liên tục 0,60 s, hệ thống chọn lại tam giác, kể cả khi leader đã đến goal. Nếu chưa đủ rộng thì giữ hàng dọc. Thời điểm dừng của ba robot còn phụ thuộc sai số bám và vật cản thực tế.

Các trạng thái trong terminal:

- `triangle / tracking`: đang bám tam giác trong vùng đủ rộng.
- `column / tracking`: đang bám hoặc nhập hàng dọc.
- `... / narrow passage`: đường leader hoặc phần hàng dọc phía sau còn cần tốc độ khe hẹp.
- `... / closing gaps`: đang giảm tốc phía trước để thu khoảng cách hàng dọc.
- `... / final slots`: đã cố định vị trí kết thúc của các follower trong hàng dọc.
- `... / leader path blocked`: bước tham chiếu leader chưa vượt qua được kiểm tra an toàn.
- `yield / follower clears leader corridor`: follower đang nhường đường.
- `yield / leader passing; followers clear corridor`: leader đang đi qua trong lúc follower nhường.

### Khi xuất hiện `leader path blocked` trong khe hẹp

Một nguyên nhân đã ghi nhận là follower lọt vào khoảng đệm tường: ở phiên bị kẹt,
`robot2` có khoảng cách tâm tới ô vật cản khoảng **0,164 m**, trong khi bộ tạo
đội hình yêu cầu **0,18 m**. MPC trước đó chỉ bắt buộc giữ bán kính thân xe
0,15 m. Vì phép kiểm tra chuyển động xét cả ba robot, vị trí follower này làm
bước đi của leader cũng bị giữ dù phía trước leader còn đường.

Cách xử lý hiện tại:

- MPC áp dụng khoảng đệm 0,18 m trên **toàn bộ đoạn chuyển động dự đoán** với
  tường và các điểm vật cản mới, thay vì chỉ yêu cầu bán kính thân xe 0,15 m.
- Nếu vị trí đo được đã lọt vào khoảng đệm do sai số bám, hệ thống phát hiện
  ngay khi đang chạy, giữ đội hình và tìm đoạn thoát ngắn cho robot đó với
  tốc độ tham chiếu 0,05 m/s. Đoạn thoát phải giữ khoảng cách thân xe, tránh
  teammate và không giảm khoảng trống đã đo; sau khi thoát, hệ thống lập lại
  đường tới goal hiện tại và tiếp tục bám đuôi.
- Log nêu đúng robot và khoảng trống, ví dụ
  `Measured obstacle clearance below 0.18 m (robot2=0.164 m)` rồi
  `Clearance recovery` / `Clearance restored`. Nếu không tìm được đoạn thoát
  an toàn hoặc khoảng trống đã nhỏ hơn bán kính thân xe, hệ thống giữ dừng và
  ghi rõ lý do.

`leader path blocked` vẫn có thể xuất hiện khi một bước đi thực sự chưa an toàn;
cần đọc cùng trạng thái robot và log phục hồi để xác định nguyên nhân.

Nếu không có chỗ né an toàn, D* thử đường vòng quanh vị trí teammate đang đứng. Khi cả chỗ né và đường vòng đều không tồn tại, nhóm phải dừng; thuật toán không bỏ kiểm tra va chạm để ép robot đi qua.

Bản mã trước sửa bám đuôi khe hẹp được lưu tại `~/ros2_ws/.codex_backups/20261010_column_following/`. Chưa có số đo thời gian di chuyển Gazebo của lần sửa này.
Bản mã trước sửa kẹt khoảng đệm tường được lưu tại `~/ros2_ws/.codex_backups/20261010_clearance_stall/`.

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

## Camera RGB-D trên leader

Launch chính bật camera cho **robot được chọn bởi `leader_robot`**, mặc định là `robot3`. Chỉ một camera RGB-D được render cho cả đội. Cụm camera có vỏ cam–đen, viền mắt kính và dải báo màu xanh; ba mắt kính là chi tiết ngoại hình, dữ liệu RGB và depth được tạo bởi một cảm biến mô phỏng đồng trục.

Tâm quang học nằm khoảng 0,264 m trên gốc `base_link`, hướng theo +X của xe và ngẩng 10°. Giá và vỏ nằm trong bán kính footprint 0,15 m hiện tại. LiDAR của leader ở z=0,330 m phía trên camera, được cấu hình **quét 91 tầng từ −60° đến 0°, bao quanh 360°**, thay cho một mặt phẳng ngang. Các tia hướng xuống bổ sung quan sát vật thấp; một node lọc point cloud tạo scan tránh vật cản cho cả D* Lite và MPC.

Mặc định: **640×480, 15 Hz, góc nhìn ngang 87°, khoảng cắt mô phỏng 0,15–8 m**. Tần số cấu hình tính theo thời gian mô phỏng; tốc độ ảnh theo thời gian thực phụ thuộc tải máy và tốc độ Gazebo. Ảnh RGB và depth cùng kích thước, cùng hệ quang học; depth có encoding `32FC1`, đơn vị mét. Khoảng cắt là cấu hình mô phỏng, không phải cam kết khoảng nhận diện người. Dữ liệu không hợp lệ hoặc ngoài tầm cần được loại bỏ trong bộ xử lý ảnh.

| Topic ROS 2 | Nội dung |
| --- | --- |
| `/leader/camera/image_raw` | Ảnh màu dùng cho bộ phát hiện người; có cửa sổ `Leader RGB Camera` trong RViz. |
| `/leader/camera/depth/image_raw` | Chiều sâu đã căn với ảnh màu, dùng để tính vị trí/khoảng cách của người sau khi có detection. |
| `/leader/camera/camera_info` | Nội tham số camera dùng chung cho ảnh màu và depth. |
| `/leader/lidar/points` | Point cloud LiDAR 240 hướng × 91 tầng, 5 Hz theo thời gian mô phỏng. |
| `/robot3/scan` | Scan 2D đã lọc độ cao, giữ vật cản gần nhất theo từng hướng; đổi prefix theo `leader_robot`. |

Các topic `/leader/...` giữ nguyên khi đổi `leader_robot:=robot1` hoặc `robot2`. Topic Gazebo tương ứng là `/leader/camera/image`, `/leader/camera/depth_image`, `/leader/camera/camera_info`. Bridge một chiều chỉ truyền ảnh khi có subscriber để giảm tải. Point cloud từ camera chưa được bridge; point cloud LiDAR được dùng riêng cho tránh vật cản thấp. Header ảnh ROS dùng `<robot>/leader_camera_optical_frame`: +Z nhìn về trước, +X sang phải, +Y xuống dưới theo [REP-103](https://www.ros.org/reps/rep-0103.html#suffix-frames). Chuỗi TF đi qua `base_link → leader_camera_mount → leader_camera_link → leader_camera_frame → leader_camera_optical_frame`, có prefix robot đúng một lần. Cấu hình RGB-D theo [ví dụ ros_gz](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/rgbd_camera_bridge.launch.py).

### Phát hiện vật cản thấp

`leader_obstacle_scan.py` nhận `/leader/lidar/points`, đổi tọa độ bằng TF về `<robot>/base_link`, bỏ điểm không hợp lệ, bỏ điểm trong bán kính thân xe 0,16 m và chỉ giữ điểm có z=0,025–0,40 m so với `base_link`. Mặt sàn của `my_maze` nằm dưới gốc thân xe nên được loại ra. Theo mỗi hướng, node lấy **khoảng cách ngang nhỏ nhất** qua các tầng để phát `/<leader>/scan`; không dùng chiều dài tia xiên làm khoảng cách tránh vật cản. Vật nằm phía sau hoặc hai bên cũng được xử lý, độc lập với góc nhìn camera.

Scan thô của Gazebo được chuyển sang `/leader/lidar/raw_scan` chỉ để chẩn đoán. Node lọc là nguồn duy nhất của scan leader đưa vào bộ điều khiển; hai follower tiếp tục dùng LiDAR 2D cũ. Point cloud quá 0,30 giây, sai frame hoặc thiếu TF sẽ bị bỏ qua. Nếu node lọc thoát, launch dừng phiên mô phỏng. Chế độ `leader_camera:=false` khôi phục LiDAR 2D thấp và bridge cũ.

Các ngưỡng độ cao phù hợp **sàn phẳng `my_maze`**. Bậc cao thấp rất nhỏ, dây mảnh, vùng bị che khuất hoặc dốc lớn vẫn cần cảm biến/ước lượng mặt đất phù hợp; một scan không có điểm không chứng minh toàn bộ thể tích là trống. Tầm phát hiện vật thấp phụ thuộc kích thước vật, khoảng cách và khoảng cách góc giữa các tia. Bộ này dùng cấu hình LiDAR nhiều tầng mà [Gazebo hỗ trợ](https://gazebosim.org/api/sensors/6/classignition_1_1sensors_1_1GpuLidarSensor.html), không giả lập khả năng 3D bằng LiDAR phần cứng chỉ quét một mặt phẳng.

Kiểm tra topic khi launch đang chạy:

```bash
ros2 topic info /robot3/scan --verbose
ros2 topic echo --once /robot3/scan --field header
```

Trong RViz có thể thêm `LaserScan`, chọn `/robot3/scan` để xem các vật cản đã được chiếu xuống mặt phẳng điều hướng. Đổi `robot3` theo leader nếu cấu hình khác.

Kiểm tra bổ sung ngày 10/10/2026: đặt bốn khối 18×18×5 cm ở trước/sau/trái/phải, tâm cách leader 0,80 m trong một phiên Gazebo riêng. Cả bốn phía trả về khoảng cách gần nhất khoảng 0,69–0,70 m, trong khi mặt sàn và thân xe không tạo vật cản giả gần xe ở lần đo nền. Camera vẫn phát ảnh. Khi đặt goal phía sau khối phía trước, D* Lite lập đường vòng; đoạn gần nhất cách tâm khối 0,301 m. Xác nhận `/robot3/scan` chỉ có một publisher và được cả node điều phối lẫn MPC robot3 nhận. Bốn kiểm tra tự động về lọc độ cao, phép đổi tọa độ, khoảng cách ngang và định dạng point cloud đều đạt. Phiên này dùng `motion_enabled:=false`: đã kiểm tra nhận cảm biến và lập đường, chưa đo va chạm khi đội hình chạy qua vật thấp.

Bản trước thay đổi quét nhiều tầng được lưu ở `~/ros2_ws/.codex_backups/20261010_low_obstacles/`.

Build và chạy như mục trên. Xem ảnh riêng trong terminal khác:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /leader/camera/image_raw
```

Xem thông số và TF (camera chỉ có trong chế độ `simulation:=true`):

```bash
ros2 topic echo --once /leader/camera/camera_info
ros2 run tf2_ros tf2_echo robot3/base_link robot3/leader_camera_optical_frame
```

Có thể tăng chi tiết ảnh nếu máy còn dư tài nguyên, hoặc giảm tải/tắt camera:

```bash
# Ảnh lớn hơn; cần đánh giá lại tốc độ mô phỏng và độ trễ xử lý ảnh.
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py camera_width:=960 camera_height:=720 camera_rate:=20

# Tốc độ ảnh thấp hơn.
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py camera_rate:=10

# Quay lại robot không có cụm RGB-D và vị trí LiDAR cũ.
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py leader_camera:=false
```

Bản này lắp và kết nối camera. Launch `three_robot_social.launch.py` đã có bộ
phát hiện người YOLOv4-tiny, ghép depth và TF để hiển thị vị trí người trên RViz;
xem [hướng dẫn nhận diện](README_SOCIAL_SCENE.md#nhận-diện-người-bằng-camera-leader).
Launch xã hội dùng thêm YuNet + PnP để ước lượng hướng đầu, track vị trí ẩn danh
và dự báo vận tốc trong 1,5 s; tạo map tránh dùng chung cho D* Lite và ba MPC.
Leader có thể quay nhìn quanh, follower giữ chỗ, rồi tiếp tục theo đường cập
nhật. Camera vẫn gắn cố định theo thân xe; quay không khắc phục được vật che
hoặc mặt người quay lưng. `my_maze` chưa có người; `social_rescue` có 8 người
đứng và 2 người ngồi tĩnh. Chưa đo độ chính xác hướng đầu, suy luận nhóm hoặc
hiệu quả điều hướng của các phần mới; không nhận dạng danh tính hay đo mắt.

Kiểm tra ngày 10/10/2026 trên Gazebo Fortress với renderer Intel, `motion_enabled:=false`: sinh đủ ba robot; nhận cả ba LiDAR, ảnh RGB `rgb8`, depth `32FC1` và CameraInfo 640×480. Trong cửa sổ lấy mẫu 30 giây, thu được 101 bộ RGB/depth/CameraInfo trùng timestamp; kiểm tra được nội tham số góc nhìn 87° và TF từ camera tới `base_link`/`map`. Đã xem ảnh camera và cụm lắp trong Gazebo. Đây là kiểm tra tích hợp cảm biến khi đứng yên, chưa đánh giá hiệu quả nhận diện người hoặc chuyển động đội hình sau khi lắp camera.

Bản trước khi gắn camera được lưu tại `~/ros2_ws/.codex_backups/20261010_leader_camera/`.

## Tùy chọn đội hình

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
ros2 launch ros2_omni_robot_sim three_robot_dstar_mpc.launch.py formation_transition_speed:=0.22 formation_follower_speed:=0.22 formation_narrow_follower_speed:=0.12 formation_open_hold:=0.60
```

`formation_transition_speed` là tốc độ tham chiếu khi follower còn cách slot hơn 0,12 m; `formation_follower_speed` là tốc độ bám thường xuyên. `formation_narrow_follower_speed` giới hạn tốc độ bắt kịp của follower trong hàng dọc ở chỗ hẹp, mặc định 0,12 m/s, không nhỏ hơn `narrow_speed` (0,08 m/s). Tốc độ áp dụng là giá trị nhỏ hơn giữa tốc độ bám/nhập và giới hạn khe hẹp. Các tham số tốc độ tính bằng m/s và không vượt `max_linear_speed` của node điều phối, mặc định 0,28 m/s. `formation_open_hold` tính bằng giây và chỉ áp dụng khi mở lại tam giác; phát hiện khe hẹp không phải chờ khoảng thời gian này. MPC vẫn có thể giảm tốc hoặc dừng nếu khoảng cách thực tế không cho phép.

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

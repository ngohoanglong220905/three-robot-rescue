#include <chrono>
#include <memory>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <stdexcept>
#include <unordered_map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

#define WHEEL_RADIUS        0.03
#define ROBOT_RADIUS        0.088
#define DEFAULT_ROBOT_MODEL "3w_v2"

std::unordered_map<std::string, int> robot_wheel_count_list = {
  {"3w", 3},
  {"3w_v2", 3},
  {"4w", 4},
  {"5w", 5},
  {"6w", 6},
};

std::unordered_map<std::string, double> robot_offset_heading_list = {
  {"3w", 0},
  {"3w_v2", 0},
  {"4w", -45},
  {"5w", 0},
  {"6w", 0},
};


class OmniKinematics : public rclcpp::Node
{
public:

  OmniKinematics(
      int num_wheels_,
      double robot_radius_,
      double wheel_radius_,
      double heading_offset_ = 0)
  : Node("omni_kinematics")
  {
    /*
     * Lấy namespace của robot.
     *
     * Ví dụ:
     *   /robot1 -> robot1
     *   /robot2 -> robot2
     *   /robot3 -> robot3
     */
    frame_prefix_ = this->get_namespace();

    if (!frame_prefix_.empty() && frame_prefix_[0] == '/') {
      frame_prefix_.erase(0, 1);
    }

    N = num_wheels_;
    R = robot_radius_;
    r = wheel_radius_;
    heading_offset = heading_offset_;

    /*
     * Ma trận động học thuận:
     * velocity robot -> velocity bánh xe
     */
    tM = init_transform_matrix(N, heading_offset_);

    /*
     * Ma trận nghịch đảo:
     * velocity bánh xe -> velocity robot
     */
    tMI = pseudo_inverse(tM);

    /*
     * Chỉ lấy 2 hàng đầu:
     * vx, vy
     */
    tMI = Eigen::MatrixXd(tMI.block(0, 0, 2, tMI.cols()));


    /*
     * Mapping joint name -> index
     */
    for (int i = 1; i < N + 1; i++) {

      string joint_name =
          "omni_wheel_joint_" + to_string(i);

      wheel_joint_map_index[joint_name] = i - 1;
    }


    /*
     * Publisher điều khiển từng bánh xe
     *
     * Topic dùng tên tương đối.
     *
     * Nếu namespace = /robot1:
     *
     * robot1/wheel1_controller/commands
     */
    for (int i = 0; i < N; i++) {

      string topic_name =
          "wheel" + to_string(i + 1) +
          "_controller/commands";

      auto pub =
          this->create_publisher<
              std_msgs::msg::Float64MultiArray>(
              topic_name,
              10);

      pub_wheels.push_back(pub);
    }


    /*
     * Odometry
     *
     * Nếu namespace = /robot1:
     *
     * /robot1/odom
     */
    pub_odometry =
        this->create_publisher<
            nav_msgs::msg::Odometry>(
            "odom",
            10);


    /*
     * /cmd_vel
     *
     * Dùng topic tương đối để sau này namespace
     * tự động tách cho robot1, robot2, robot3.
     */
    sub_cmd_vel =
        this->create_subscription<
            geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(
                &OmniKinematics::cmd_vel_callback,
                this,
                _1));


    /*
     * Joint states
     */
    sub_join_states =
        this->create_subscription<
            sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(
                &OmniKinematics::join_states_callback,
                this,
                _1));


    /*
     * IMU
     */
    sub_imu =
        this->create_subscription<
            sensor_msgs::msg::Imu>(
            "imu",
            10,
            std::bind(
                &OmniKinematics::imu_callback,
                this,
                _1));


    /*
     * TF broadcaster
     */
    tf_broadcaster =
        std::make_shared<
            tf2_ros::TransformBroadcaster>(*this);


    /*
     * Thời gian ban đầu
     */
    last_time =
        this->get_clock()->now();


    /*
     * Phần này giữ nguyên theo code gốc
     */
    double C = 2 * r / sqrt(3);

    double h = 0 * M_PI / 180;

    mOd[0][0] =
        C * (
            cos(2 * M_PI / 3 + h)
            -
            (
              cos(2 * M_PI / 3 + h)
              - cos(h)
            ) / 3
        );

    mOd[0][1] =
        C * (
            -cos(h)
            -
            (
              cos(2 * M_PI / 3 + h)
              - cos(h)
            ) / 3
        );

    mOd[0][2] =
        C * (
            -(
              cos(2 * M_PI / 3 + h)
              - cos(h)
            ) / 3
        );

    mOd[1][0] =
        C * (
            sin(2 * M_PI / 3 + h)
            -
            (
              sin(2 * M_PI / 3 + h)
              - sin(h)
            ) / 3
        );

    mOd[1][1] =
        C * (
            -sin(h)
            -
            (
              sin(2 * M_PI / 3 + h)
              - sin(h)
            ) / 3
        );

    mOd[1][2] =
        C * (
            -(
              sin(2 * M_PI / 3 + h)
              - sin(h)
            ) / 3
        );
  }


  ~OmniKinematics()
  {
    cout << "OmniKinematics Destroyed" << endl;
  }


private:

  /*
   * ----------------------------------------------------
   * ROS 2 publishers / subscribers
   * ----------------------------------------------------
   */

  rclcpp::TimerBase::SharedPtr timer_;

  vector<
      rclcpp::Publisher<
          std_msgs::msg::Float64MultiArray>::SharedPtr>
      pub_wheels;

  rclcpp::Publisher<
      nav_msgs::msg::Odometry>::SharedPtr
      pub_odometry;

  rclcpp::Subscription<
      geometry_msgs::msg::Twist>::SharedPtr
      sub_cmd_vel;

  rclcpp::Subscription<
      sensor_msgs::msg::JointState>::SharedPtr
      sub_join_states;

  rclcpp::Subscription<
      sensor_msgs::msg::Imu>::SharedPtr
      sub_imu;

  std::shared_ptr<
      tf2_ros::TransformBroadcaster>
      tf_broadcaster;


  /*
   * ----------------------------------------------------
   * Kinematics matrices
   * ----------------------------------------------------
   */

  Eigen::MatrixXd tM;

  Eigen::MatrixXd tMI;

  unordered_map<string, int>
      wheel_joint_map_index;


  /*
   * ----------------------------------------------------
   * Robot parameters
   * ----------------------------------------------------
   */

  int N;

  double r;

  double R;

  double heading_offset;


  /*
   * Namespace của robot
   *
   * robot1
   * robot2
   * robot3
   */
  std::string frame_prefix_;


  /*
   * ----------------------------------------------------
   * Robot state
   * ----------------------------------------------------
   */

  double mOd[2][3] = {
    {0, 0, 0},
    {0, 0, 0}
  };

  double pos_x = 0;

  double pos_y = 0;

  double vx = 0;

  double vy = 0;

  double pitch = 0;

  double roll = 0;

  double yaw = 0;

  double w = 0;


  /*
   * Thời gian lần callback trước
   */
  rclcpp::Time last_time;


  /*
   * ----------------------------------------------------
   * Tạo tên frame theo namespace
   *
   * Ví dụ:
   *
   * robot1 + odom
   * -> robot1/odom
   *
   * robot2 + base_footprint
   * -> robot2/base_footprint
   * ----------------------------------------------------
   */
  std::string frame_name(
      const std::string &name)
  {
    if (frame_prefix_.empty()) {
      return name;
    }

    return frame_prefix_ + "/" + name;
  }


  /*
   * ----------------------------------------------------
   * Cmd_vel callback
   * ----------------------------------------------------
   */
  void cmd_vel_callback(
      const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    Eigen::VectorXd M =
        calculate_motor_speed(
            msg->linear.x,
            msg->linear.y,
            msg->angular.z);

    set_motor_speed(M);
  }


  /*
   * ----------------------------------------------------
   * Joint state callback
   * ----------------------------------------------------
   */
  void join_states_callback(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {

    /*
     * Khởi tạo bằng 0 để tránh giá trị rác
     */
    Eigen::VectorXd wheel_velocity =
        Eigen::VectorXd::Zero(N);


    /*
     * Lấy velocity của từng bánh
     */
    for (size_t i = 0;
         i < msg->name.size();
         ++i)
    {

      for (const auto& pair :
           wheel_joint_map_index)
      {

        if (msg->name[i] == pair.first)
        {

          if (i < msg->velocity.size())
          {
            wheel_velocity(pair.second) =
                msg->velocity[i];
          }

          break;
        }
      }
    }


    /*
     * Thời gian
     */
    rclcpp::Time current_time =
        this->get_clock()->now();

    double dt =
        (current_time - last_time).seconds();

    last_time = current_time;


    /*
     * Không tích phân nếu dt không hợp lệ
     */
    if (dt <= 0.0 || dt > 1.0)
    {
      publish_odom(current_time);
      return;
    }


    /*
     * Tính vận tốc trong hệ robot
     *
     * [vx_body]
     * [vy_body]
     *
     * = tMI * wheel_velocity
     */
    Eigen::VectorXd body_velocity =
        tMI * wheel_velocity;


    /*
     * Lưu vận tốc robot
     */
    vx = body_velocity(0);

    vy = body_velocity(1);


    /*
     * Chuyển từ hệ robot sang hệ world
     */
    Eigen::Matrix2d rM;

    rM <<
        cos(yaw), -sin(yaw),
        sin(yaw),  cos(yaw);


    Eigen::VectorXd world_velocity =
        rM * body_velocity;


    /*
     * Tích phân vị trí
     */
    pos_x +=
        world_velocity(0) * dt;

    pos_y +=
        world_velocity(1) * dt;


    /*
     * Publish odometry
     */
    publish_odom(current_time);
  }


  /*
   * ----------------------------------------------------
   * IMU callback
   * ----------------------------------------------------
   */
  void imu_callback(
      const sensor_msgs::msg::Imu::SharedPtr msg)
  {

    /*
     * Angular velocity
     */
    w =
        msg->angular_velocity.z;


    /*
     * Quaternion -> RPY
     */
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );


    tf2::Matrix3x3 m(q);


    m.getRPY(
        roll,
        pitch,
        yaw
    );
  }


  /*
   * ----------------------------------------------------
   * Publish odometry + TF
   * ----------------------------------------------------
   */
  void publish_odom(
      rclcpp::Time current_time)
  {

    nav_msgs::msg::Odometry odom_msg;


    /*
     * Frame có namespace
     */
    odom_msg.header.stamp =
        current_time;

    odom_msg.header.frame_id =
        frame_name("odom");

    odom_msg.child_frame_id =
        frame_name("base_footprint");


    /*
     * ------------------------------------------------
     * Position
     * ------------------------------------------------
     */
    odom_msg.pose.pose.position.x =
        pos_x;

    odom_msg.pose.pose.position.y =
        pos_y;

    odom_msg.pose.pose.position.z =
        0.0;


    /*
     * ------------------------------------------------
     * Orientation
     * ------------------------------------------------
     */
    tf2::Quaternion q;

    q.setRPY(
        0,
        0,
        yaw
    );


    odom_msg.pose.pose.orientation.x =
        q.x();

    odom_msg.pose.pose.orientation.y =
        q.y();

    odom_msg.pose.pose.orientation.z =
        q.z();

    odom_msg.pose.pose.orientation.w =
        q.w();


    /*
     * ------------------------------------------------
     * Velocity
     * ------------------------------------------------
     */
    odom_msg.twist.twist.linear.x =
        vx;

    odom_msg.twist.twist.linear.y =
        vy;

    odom_msg.twist.twist.angular.z =
        w;


    /*
     * Publish odometry
     */
    pub_odometry->publish(
        odom_msg);


    /*
     * ------------------------------------------------
     * Publish TF
     *
     * robot1/odom
     *       |
     *       v
     * robot1/base_footprint
     * ------------------------------------------------
     */
    geometry_msgs::msg::TransformStamped odom_tf;


    odom_tf.header.stamp =
        current_time;

    odom_tf.header.frame_id =
        frame_name("odom");

    odom_tf.child_frame_id =
        frame_name("base_footprint");


    odom_tf.transform.translation.x =
        pos_x;

    odom_tf.transform.translation.y =
        pos_y;

    odom_tf.transform.translation.z =
        0.0;


    odom_tf.transform.rotation =
        odom_msg.pose.pose.orientation;


    tf_broadcaster->sendTransform(
        odom_tf);
  }


  /*
   * ----------------------------------------------------
   * Tính tốc độ bánh
   *
   * robot velocity
   *        ↓
   * wheel velocity
   * ----------------------------------------------------
   */
  Eigen::VectorXd calculate_motor_speed(
      float x_,
      float y_,
      float w_)
  {

    Eigen::Vector3d v(
        x_,
        y_,
        w_
    );


    Eigen::VectorXd M =
        tM * v;


    return M;
  }


  /*
   * ----------------------------------------------------
   * Publish command cho bánh
   * ----------------------------------------------------
   */
  void set_motor_speed(
      Eigen::VectorXd M)
  {

    for (int i = 0;
         i < M.size();
         i++)
    {

      auto message =
          std_msgs::msg::Float64MultiArray();


      message.data = {
        M(i)
      };


      pub_wheels[i]->publish(
          message);
    }
  }


  /*
   * ----------------------------------------------------
   * Tạo ma trận động học
   * ----------------------------------------------------
   */
  Eigen::MatrixXd init_transform_matrix(
      int N,
      double heading_offset = 0)
  {

    Eigen::MatrixXd M =
        Eigen::MatrixXd::Zero(
            N,
            3);


    double del_angle =
        360.0 / N;


    for (int i = 0;
         i < N;
         i++)
    {

      M(i, 0) =
          -sin(
              (del_angle * i +
               heading_offset)
              * M_PI / 180
          ) / r;


      M(i, 1) =
          cos(
              (del_angle * i +
               heading_offset)
              * M_PI / 180
          ) / r;


      M(i, 2) =
          R / r;
    }


    return M;
  }


  /*
   * ----------------------------------------------------
   * Pseudo inverse
   * ----------------------------------------------------
   */
  Eigen::MatrixXd pseudo_inverse(
      const Eigen::MatrixXd& A,
      double tolerance = 1e-8)
  {

    Eigen::JacobiSVD<
        Eigen::MatrixXd>
        svd(
            A,
            Eigen::ComputeThinU |
            Eigen::ComputeThinV
        );


    const auto& singularValues =
        svd.singularValues();


    Eigen::MatrixXd S_inv =
        Eigen::MatrixXd::Zero(
            svd.matrixV().cols(),
            svd.matrixU().cols()
        );


    for (int i = 0;
         i < singularValues.size();
         ++i)
    {

      if (singularValues(i) >
          tolerance)
      {

        S_inv(i, i) =
            1.0 /
            singularValues(i);
      }
    }


    return
        svd.matrixV()
        *
        S_inv
        *
        svd.matrixU().transpose();
  }
};


/*
 * =====================================================
 * MAIN
 * =====================================================
 */
int main(
    int argc,
    char * argv[])
{

  rclcpp::init(
      argc,
      argv);


  /*
   * Lấy robot model từ environment
   */
  const char* env_robot_model =
      std::getenv(
          "OMNI_ROBOT_MODEL");


  std::string robot_model =
      env_robot_model
      ?
      env_robot_model
      :
      DEFAULT_ROBOT_MODEL;


  /*
   * Kiểm tra model
   */
  if (
      robot_wheel_count_list.find(
          robot_model)
      ==
      robot_wheel_count_list.end()
  )
  {

    std::string msg =
        "Unknown robot model: "
        +
        robot_model
        +
        "\nValid options are:\n";


    for (
        const auto& entry :
        robot_wheel_count_list)
    {

      msg +=
          "- "
          +
          entry.first
          +
          "\n";
    }


    throw std::runtime_error(
        msg);
  }


  /*
   * Số bánh
   */
  int wheel_count =
      robot_wheel_count_list[
          robot_model
      ];


  /*
   * Chạy node
   */
  rclcpp::spin(
      std::make_shared<
          OmniKinematics>(
          wheel_count,
          ROBOT_RADIUS,
          WHEEL_RADIUS,
          robot_offset_heading_list[
              robot_model
          ]
      )
  );


  rclcpp::shutdown();

  return 0;
}
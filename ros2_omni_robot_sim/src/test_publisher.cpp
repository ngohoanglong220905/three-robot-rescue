#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cpp_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  rclcpp::Rate rate(1); // 1 Hz
  while (rclcpp::ok()) {
    std_msgs::msg::String msg;
    msg.data = "Hello from C++";
    publisher->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published: '%s'", msg.data.c_str());
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
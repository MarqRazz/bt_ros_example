#include <ros_node/ros_node.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto node = std::make_shared<bt_ros_example::AwesomeROSNode>(options);
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->getNodeBaseInterface());
  node->run_tree(&exec);
  exec.remove_node(node->getNodeBaseInterface());

  rclcpp::shutdown();
}

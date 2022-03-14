#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <bt_user_input/subscriber_input.hpp>
#include <bt_user_input/subscriber_input_coro.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
// other nice to haves
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

namespace bt_ros_example
{
class AwesomeROSNode
{
public:
  explicit AwesomeROSNode(const rclcpp::NodeOptions& options);

  /**
   * @brief Getter for the underlying Node objects base interface.
   * @return Shared pointer Node's internal NodeBaseInterface implementation.
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void initialize();
  void run_tree(rclcpp::Executor *executor);


private:
  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

}  // namespace agent_vision

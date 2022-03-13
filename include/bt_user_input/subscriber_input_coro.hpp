// Copyright 2021 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
using namespace BT;
using namespace std::placeholders;

namespace bt_ros_example_coro
{  
class SubscriberInput: public CoroActionNode
{
public:
  SubscriberInput(const std::string& name, const NodeConfiguration& config,
                 rclcpp::Node::SharedPtr node, std::string input_topic);

  NodeStatus tick() override;
  void halt() override;

  static PortsList providedPorts()
  {
    return { };
  }

  std::string input_topic_;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::atomic<bool> msg_received_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr empty_msg_sub_;

  void subscriberCb(const std_msgs::msg::Empty::SharedPtr msg);  
};

}  // namespace bt_ros_example_coro

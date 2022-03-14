#include <bt_user_input/subscriber_input.hpp>

// to trigger this subscriber call: ros2 topic pub /some_msg std_msgs/msg/Empty {}

static const auto LOGGER = rclcpp::get_logger("subscriber_input");

namespace bt_ros_example_stateful
{
SubscriberInput::SubscriberInput(const std::string& name, const NodeConfiguration& config,
             rclcpp::Node::SharedPtr node, std::string input_topic) : 
             StatefulActionNode(name, config), node_(node), input_topic_(input_topic)
{
  RCLCPP_INFO(LOGGER, "SubscriberInput Stateful constructor");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = callback_group_;

  empty_msg_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    input_topic_, rclcpp::SensorDataQoS().reliable(),
    std::bind(&SubscriberInput::subscriberCb, this, std::placeholders::_1), options);
}

NodeStatus SubscriberInput::onStart()
{
  RCLCPP_INFO(LOGGER, "Start waiting for message", input_topic_.c_str());
  msg_received_ = false;
  return NodeStatus::RUNNING;
}

NodeStatus SubscriberInput::onRunning()
{
  if(msg_received_)
  {
    RCLCPP_INFO(LOGGER,"Received msg from: %s", input_topic_.c_str());
    return NodeStatus::SUCCESS;
  }
  
  return NodeStatus::RUNNING;
}

void SubscriberInput::onHalted()
{
  RCLCPP_ERROR(LOGGER, "onHalted() has been called");
}

void SubscriberInput::subscriberCb(const std_msgs::msg::Empty::SharedPtr msg)
{
  msg_received_ = true;
  RCLCPP_WARN(LOGGER,"Received msg from callback");
}
}  // namespace bt_ros_example

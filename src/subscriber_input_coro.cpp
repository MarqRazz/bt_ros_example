#include <bt_user_input/subscriber_input_coro.hpp>

// to trigger this subscriber call: ros2 topic pub /some_msg std_msgs/msg/Empty {}

static const auto LOGGER = rclcpp::get_logger("subscriber_input");

namespace bt_ros_example_coro
{
SubscriberInput::SubscriberInput(const std::string& name, const NodeConfiguration& config,
             rclcpp::Node::SharedPtr node, std::string input_topic) : 
             CoroActionNode(name, {}), node_(node), input_topic_(input_topic)
{
  RCLCPP_INFO(LOGGER, "SubscriberInput CoroActionNode constructor");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = callback_group_;
  RCLCPP_INFO(LOGGER, "Subscribing to Input Topic: %s", input_topic_.c_str());
  msg_received_ = false;
  empty_msg_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    input_topic_, rclcpp::SensorDataQoS().reliable(), 
    std::bind(&SubscriberInput::subscriberCb, this, std::placeholders::_1), options);
}

NodeStatus SubscriberInput::tick()
{
  auto Now = [](){ return std::chrono::high_resolution_clock::now(); };

  TimePoint initial_time = Now();
  TimePoint time_before_reply = initial_time + std::chrono::milliseconds(10000);

  while( !msg_received_ )
  {
    RCLCPP_INFO_ONCE(LOGGER, "Waiting for subscriber msg");
    if( !msg_received_ )
    {
      if( Now() >= time_before_reply )
      {
        RCLCPP_ERROR(LOGGER, "Failed to receive msg after 10 seconds, self calling CB");
        std_msgs::msg::Empty::SharedPtr msg;
        subscriberCb(msg);
      }
      // set status to RUNNING and "pause/sleep"
      // If halt() is called, we will not resume execution (stack destroyed)
      setStatusRunningAndYield();
    }
  }

  RCLCPP_INFO(LOGGER,"Received msg from: %s", input_topic_.c_str());
  return NodeStatus::SUCCESS;
}

void SubscriberInput::halt()
{
  RCLCPP_ERROR(LOGGER, "halt() has been called");
  empty_msg_sub_.reset();
  // Do not forget to call this at the end.
  CoroActionNode::halt();
}

void SubscriberInput::subscriberCb(const std_msgs::msg::Empty::SharedPtr msg)
{
  msg_received_ = true;
  RCLCPP_WARN(LOGGER,"Received msg from callback");
}
}  // namespace bt_ros_example_coro

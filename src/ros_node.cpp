
#include <ros_node/ros_node.hpp>

static const auto LOGGER = rclcpp::get_logger("bt_test_node");

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SubscriberInput name="subscriber_input"/>           
        </Sequence>
     </BehaviorTree>

 </root>
 )";

// clang-format on

namespace bt_ros_example
{
constexpr auto kNodeName = "bt_test_node";
AwesomeROSNode::AwesomeROSNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>(kNodeName, options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr AwesomeROSNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void AwesomeROSNode::initialize()
{
  auto node_ptr = node_->shared_from_this();
  std::string topic = "some_msg";

  NodeBuilder user_input_builder = [node_ptr, topic](const std::string& name, const NodeConfiguration& config)
  {
      return std::make_unique<bt_ros_example_stateful::SubscriberInput>( name, config, node_ptr, topic);
  };
  
  NodeBuilder user_input_builder_coro = [node_ptr, topic](const std::string& name, const NodeConfiguration& config)
  {
      return std::make_unique<bt_ros_example_coro::SubscriberInput>( name, config, node_ptr, topic);
  };
  
  factory_.registerBuilder<bt_ros_example_coro::SubscriberInput>( "SubscriberInput", user_input_builder);
  factory_.registerBuilder<bt_ros_example_coro::SubscriberInput>( "SubscriberInputCoro", user_input_builder_coro);
}

void AwesomeROSNode::run_tree()
{
  RCLCPP_WARN(LOGGER, "creating Tree");
  auto tree = factory_.createTreeFromText(xml_text);

  // This logger prints state changes on console
  StdCoutLogger logger_cout(tree);
  PublisherZMQ publisher_zmq(tree);

  //---------------------------------------
  // keep executin tick until it returns etiher SUCCESS or FAILURE
  NodeStatus status = NodeStatus::RUNNING;
  while( status == NodeStatus::RUNNING && rclcpp::ok())
  {
    status = tree.tickRoot(); 
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }
}
}  // namespace bt_ros_example

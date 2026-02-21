#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

/**
 * This is an example - you can remove/modify it and implement your own node.
 */
namespace example {
class EchoNode : public rclcpp::Node {
public:
  EchoNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("echo", options),
    _publisher(this->create_publisher<std_msgs::msg::String>(
      "~/output", rclcpp::QoS(1))),
    _subscription(this->create_subscription<std_msgs::msg::String>(
      "~/input", rclcpp::QoS(1),
      std::bind(&EchoNode::input_callback, this, std::placeholders::_1))) { }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;

  void input_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    auto echoed_msg = std_msgs::msg::String();
    echoed_msg.data = "Received: " + msg->data;
    _publisher->publish(echoed_msg);
  }
};
}  // namespace example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(example::EchoNode)

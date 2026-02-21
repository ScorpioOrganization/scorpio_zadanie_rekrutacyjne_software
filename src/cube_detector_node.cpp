#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

namespace cube_detector {
class CubeDetectorNode : public rclcpp::Node {
public:
  CubeDetectorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("cube_detector_node", options),
    _publisher(this->create_publisher<std_msgs::msg::String>(
      "~/echoed", rclcpp::QoS(1))),
    _subscription(this->create_subscription<std_msgs::msg::String>(
      "~/echo", rclcpp::QoS(1),
      std::bind(&CubeDetectorNode::echo_callback, this, std::placeholders::_1))) { }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;

  void echo_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    auto echoed_msg = std_msgs::msg::String();
    echoed_msg.data = "Echo: " + msg->data;
    _publisher->publish(echoed_msg);
  }
};
}  // namespace cube_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cube_detector::CubeDetectorNode)

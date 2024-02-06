
#include "rb_connector.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbRobot>();

  rclcpp::WallRate loop_rate(100);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->updateJoint();
    loop_rate.sleep();
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

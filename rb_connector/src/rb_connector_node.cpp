
#include "rb_connector.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbRobot>();

  rclcpp::WallRate loop_rate(100);  // 2Hz

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    // 예: UpdateJoint();
    loop_rate.sleep();
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

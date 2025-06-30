#include "s1_driver/s1_driver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<s1_driver::S1Driver>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}

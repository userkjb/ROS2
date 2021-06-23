#include "rclcpp/rclcpp.hpp"
#include "ex_service_client/ExServiceClient.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExServiceClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

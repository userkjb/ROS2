#include "rclcpp/rclcpp.hpp"
#include "ex_service_server/ExServiceServer.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExServiceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto a_node = std::make_shared<"AClass">();
    auto b_node = std::make_shared<"BClass">();

    //rclcpp::spin(node);
    executor.add_node(a_node);
    executor.add_node(b_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
*/

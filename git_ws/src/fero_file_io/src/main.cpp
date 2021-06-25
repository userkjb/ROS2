#include "rclcpp/rclcpp.hpp"
#include "fero_file_io/Fero_File_IO.hpp" // new class header

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Fero_File_IO>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

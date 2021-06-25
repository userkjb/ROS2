#ifndef ARITHMETIC__FERO_FILE_IO_HPP_
#define ARITHMETIC__FERO_FILE_IO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "action_msg_srv/srv/log_file_info.hpp"

#include <dirent.h>

using namespace std::chrono_literals; // 1s

class Fero_File_IO : public rclcpp::Node
{
public :
  Fero_File_IO();
  virtual ~Fero_File_IO();
private :
  // booting
  void Service_Function();

  // Publisher Timer
  void Pub_Timer();

  // Service Client
  void Create_Service();
  void Read_file();
private :
  // Publisher Timer
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fero_log_;

  // Service Server
  rclcpp::Service<action_msg_srv::srv::LogFileInfo>::SharedPtr log_file_server_;

protected :
  const std::string log_dir_ = "/home/fero/catkin_ws/log/....";
};


#endif

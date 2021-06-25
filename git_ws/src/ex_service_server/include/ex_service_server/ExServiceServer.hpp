#ifndef ARITHMETIC__EX_SERVICE_SERVER_HPP_
#define ARITHMETIC__EX_SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/log_file_info.hpp"

class ExServiceServer : public rclcpp::Node
{
public :
  ExServiceServer();
  virtual ~ExServiceServer();

  using LogSRV = action_msg_srv::srv::LogFileInfo;

private :
  // Callback Function
  // double RecvCallBack();
  // void SendCallBack();

private :
  std::string response_str_base_path_;
  std::string response_str_file_name_;
  rclcpp::Service<LogSRV>::SharedPtr handle_server_;
};


#endif

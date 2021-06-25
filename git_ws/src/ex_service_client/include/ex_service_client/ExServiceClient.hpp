#ifndef ARITHMETIC__EX_SERVICE_CLIENT_HPP_
#define ARITHMETIC__EX_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/log_file_info.hpp"


using namespace std::chrono_literals; // 1s

class ExServiceClient : public rclcpp::Node
{
public :
  using LogSRV = action_msg_srv::srv::LogFileInfo;

  explicit ExServiceClient(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  virtual ~ExServiceClient();

private :
  void send_request();
  void timer_function();
private :
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<action_msg_srv::srv::LogFileInfo>::SharedPtr handle_client_;
};


#endif

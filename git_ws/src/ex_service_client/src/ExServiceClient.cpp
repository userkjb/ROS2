#include "ex_service_client/ExServiceClient.hpp"

ExServiceClient::ExServiceClient(const rclcpp::NodeOptions& node_options)
  : Node("ex_service_client", node_options)
{
  handle_client_ = this->create_client<action_msg_srv::srv::LogFileInfo>("log_service");

  timer_ = this->create_wall_timer(1s, std::bind(&ExServiceClient::timer_function, this));

  while(!handle_client_->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
}

ExServiceClient::~ExServiceClient()
{
  std::cout << "end" << std::endl;
}

void ExServiceClient::timer_function()
{
  send_request();
}


void ExServiceClient::send_request()
{
  auto request = std::make_shared<LogSRV::Request>();

  // request->string = "asdf";

  using ServiceResponseFuture = rclcpp::Client<LogSRV>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future){
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Result base_path : %s", response->base_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Result file_name : %s", response->file_name.c_str());
    return;
  };

  auto future_result = handle_client_->async_send_request(request, response_received_callback);
}

#include "ex_service_server/ExServiceServer.hpp"

ExServiceServer::ExServiceServer() : Node("ex_service_server")
{
  std::cout << "111" << std::endl;
  response_str_base_path_ = "path";
  response_str_file_name_ = "file";
  auto recv_request = [this](
    const std::shared_ptr<LogSRV::Request> request,
    std::shared_ptr<LogSRV::Response> response) -> void
    {
      // response_str_base_path_ = request-> ;
      // response_str_file_name_ = request-> ;

      if(request == NULL) std::cout << "null" << std::endl;
      response->base_path = response_str_base_path_;
      response->file_name = response_str_file_name_;

      RCLCPP_INFO(this->get_logger(), "base_path = %s", response->base_path.c_str());
      RCLCPP_INFO(this->get_logger(), "file_name = %s", response->file_name.c_str());
    };

  handle_server_ = create_service<LogSRV>("log_service", recv_request);
}

ExServiceServer::~ExServiceServer()
{

}

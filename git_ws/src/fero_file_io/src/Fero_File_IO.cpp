#include "fero_file_io/Fero_File_IO.hpp"

using std::placeholders::_1;

Fero_File_IO::Fero_File_IO() : Node("fero_file_io")
{
  pub_timer_ = this->create_wall_timer(1s, std::bind(&Fero_File_IO::Pub_Timer, this));

  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  fero_log_ = this->create_publisher<std_msgs::msg::String>("logTopic", qos_profile);

  Create_Service();
}

Fero_File_IO::~Fero_File_IO()
{
  RCLCPP_INFO(this->get_logger(), "End Fero_File_IO Node");
}

// Service =========================================================
void Fero_File_IO::Create_Service()
{
  auto recv_request = [this](
    const std::shared_ptr<action_msg_srv::srv::LogFileInfo::Request> request,
    std::shared_ptr<action_msg_srv::srv::LogFileInfo::Response> response) -> void
    {
      if(request == NULL){
        std::cout << "not commant" << std::endl;
      }
      response->base_path = log_dir_;
      response->file_name = "";
    };

  log_file_server_ = this->create_service<action_msg_srv::srv::LogFileInfo>("logFile_service", recv_request);
}

void Fero_File_IO::Read_file()
{
  DIR* dirp = opendir(log_dir_.c_str());
  struct dirent* dp;

  while((dp = readdir(dirp)) != NULL)
  {
    // dp->d_name
  }
  closedir(dirp);
}


// Timer =========================================================
// Log Publisher Timer(1s)
void Fero_File_IO::Pub_Timer()
{

  // fero_log_->publish();
}

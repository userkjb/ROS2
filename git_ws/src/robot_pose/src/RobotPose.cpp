#include "robot_pose/RobotPose.hpp"

using std::placeholders::_1;

RobotPose::RobotPose() : Node("RobotPose")
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  SubSetting(qos_profile);
  PubSetting(qos_profile);
}

RobotPose::~RobotPose()
{
  RCLCPP_INFO(this->get_logger(), "End RobotPose Node");
}


void RobotPose::SubSetting(rclcpp::QoS qos)
{
  robot_Pose_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "tf",
    qos,
    std::bind(&RobotPose::TF2CallBack , this, _1)
  );

  rclcpp::QoS qos_foot(10);
  qos_foot.reliable();
  qos_foot.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  geo_pose_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "global_costmap/published_footprint",
    qos_foot,
    std::bind(&RobotPose::GlobalCostMapCallBack, this, _1)
  );
}

void RobotPose::PubSetting(rclcpp::QoS qos)
{
  pub_robot_position_ = this->create_publisher<geometry_msgs::msg::Transform>("robot_position", qos);
}

void RobotPose::TF2CallBack(const tf2_msgs::msg::TFMessage::SharedPtr tfmsg)
{
  std::string frameID = tfmsg->transforms[0].header.frame_id;
  std::string childID = tfmsg->transforms[0].child_frame_id;
  if(frameID == "odom" && childID == "base_footprint")
  {
    // std::cout << "=======================" << std::endl;
    // std::cout << "x   : " << tfmsg->transforms[0].transform.translation.x << std::endl;
    // std::cout << "y   : " << tfmsg->transforms[0].transform.translation.y << std::endl;
    // std::cout << "-----------------------" << std::endl;
    // std::cout << "r_z : " << tfmsg->transforms[0].transform.rotation.z << std::endl;

    robot_position_.rotation.z = tfmsg->transforms[0].transform.rotation.z;
    robot_position_.rotation.w = tfmsg->transforms[0].transform.rotation.w;
  }
}

void RobotPose::GlobalCostMapCallBack(const geometry_msgs::msg::PolygonStamped::SharedPtr gm)
{
  double x, y;
  int polygon_count = gm->polygon.points.size();

  for(int i = 0; i < polygon_count; i++){
    x += gm->polygon.points[i].x;
    y += gm->polygon.points[i].y;
  }
  x = x / polygon_count;
  y = y / polygon_count;

  // std::cout << "***********************" << std::endl;
  // std::cout << "x : " << x << std::endl;
  // std::cout << "y : " << y << std::endl;
  robot_position_.translation.x = x;
  robot_position_.translation.y = y;

  pub_robot_position_->publish(robot_position_);
  std::cout << "********************************" << std::endl;
  std::cout << "push translation x : " << robot_position_.translation.x << std::endl;
  std::cout << "push translation y : " << robot_position_.translation.y << std::endl;
  std::cout << "push rotation    z : " << robot_position_.rotation.z << std::endl;
  std::cout << "push rotation    w : " << robot_position_.rotation.w << std::endl;
}

#ifndef ARITHMETIC__ROBOTPOSE_HPP_
#define ARITHMETIC__ROBOTPOSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

class RobotPose : public rclcpp::Node
{
public :
  RobotPose();
  virtual ~RobotPose();
private :
  void SubSetting(rclcpp::QoS qos);
  void PubSetting(rclcpp::QoS qos);

  void TF2CallBack(const tf2_msgs::msg::TFMessage::SharedPtr tfmsg);
  void GlobalCostMapCallBack(const geometry_msgs::msg::PolygonStamped::SharedPtr gm);
private :
  // Subscriber
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_Pose_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr geo_pose_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr pub_robot_position_;

  //
  geometry_msgs::msg::Transform robot_position_;
};

#endif

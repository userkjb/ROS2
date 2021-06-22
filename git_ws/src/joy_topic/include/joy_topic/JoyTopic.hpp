#ifndef ARITHMETIC__JOYTOPIC_HPP_
#define ARITHMETIC__JOYTOPIC_HPP_

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "msg_srv/msg/driving_command.hpp"

using namespace sensor_msgs::msg; // Joy
using namespace geometry_msgs::msg; // Twist_with_covariance_stamped, Twist

struct ButtonOnOff{
    bool x_btn;
    bool rb_btn;
};

class JoyTopic : public rclcpp::Node
{
public :
    JoyTopic();
    ~JoyTopic();
private :
    ButtonOnOff buttonOnOff;
    void TopicSetting();
    void SubJoyBtn(const Joy::SharedPtr msg); // msg->joy
    void SubTwistJoy(const Twist::SharedPtr joyt);
    void WebJoyTopic(const std_msgs::msg::Float64MultiArray::SharedPtr web);
    void WebStopBtnCB(const std_msgs::msg::String::SharedPtr btn);
    void LoggerBtnText(const Joy::SharedPtr joy);
    void LoggerAxesText(const Joy::SharedPtr joy);

    double Mapping(double x, double in_min, double in_max, double out_min, double out_max);

    // Subscriber
    rclcpp::Subscription<Joy>::SharedPtr joyTopic_; // Joy 서브.(노말, 파워 버튼 On/Off Check)
    rclcpp::Subscription<Twist>::SharedPtr joyTwistTopic_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr webJoyTopic_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr webStopBtnTopic_;

    // Publisher
    rclcpp::Publisher<Joy>::SharedPtr joyTopicPub_;
    rclcpp::Publisher<Twist>::SharedPtr turtlebot3Pub_;
    rclcpp::Publisher<msg_srv::msg::DrivingCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr webStopBtn_pub_;

    // messsage
    msg_srv::msg::DrivingCommand fw_msg;
    geometry_msgs::msg::Twist tw_msg;

    // const double lowSpeed = -0.5; // 낮은 스피드
    // const double highSpeed = 0.5; // 높은 스피드
    const double lowSpeed = -0.2; // 낮은 스피드 turtlebot3
    const double highSpeed = 0.2; // 높은 스피드 turtlebot3
    const double powerLowSpeed = -0.86; // 파워 낮은 스피드
    const double powerHighSpeed = 0.86; // 파워 높은 스피드
    const double lowTurn = -0.4; // 낮은 회전
    const double highTurn = 0.4; // 높은 회전
    const double powerLowTurn = -0.6; // 파워 낮은 회전
    const double powerHighTurn = 0.6; // 파워 높은 회전
};

#endif

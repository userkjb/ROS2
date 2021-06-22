#include "joy_topic/JoyTopic.hpp"

using std::placeholders::_1;

JoyTopic::JoyTopic() : Node("joyTopic")
{
    buttonOnOff.x_btn = false;
    buttonOnOff.rb_btn = false;

    TopicSetting();
}
void JoyTopic::TopicSetting(){
    RCLCPP_INFO(this->get_logger(), "start");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    rclcpp::QoS webQos(10);
    webQos.best_effort();
    webQos.durability_volatile();

    // Subscriber============================================================
    joyTopic_ = this->create_subscription<Joy>(
        "joy",
        qos_profile,
        std::bind(&JoyTopic::SubJoyBtn, this, _1)
    );
    joyTwistTopic_ = this->create_subscription<Twist>(
        // "cmd_vel",
        "mobility/cmd_vel",
        qos_profile,
        std::bind(&JoyTopic::SubTwistJoy, this, _1)
    );
    webJoyTopic_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joystick",
        qos_profile,
        std::bind(&JoyTopic::WebJoyTopic, this, _1)
    );
    webStopBtnTopic_ = this->create_subscription<std_msgs::msg::String>(
        "test_cmd",
        qos_profile,
        std::bind(&JoyTopic::WebStopBtnCB, this, _1)
    );

    // Publisher============================================================
    joyTopicPub_ = this->create_publisher<Joy>("joyBtn", qos_profile);
    turtlebot3Pub_ = this->create_publisher<Twist>("cmd_vel", qos_profile);
    cmd_pub_ = this->create_publisher<msg_srv::msg::DrivingCommand>("/fero/cmd_vel", qos_profile);
} // end TopicSetting()

JoyTopic::~JoyTopic(){
    RCLCPP_INFO(this->get_logger(), "joy end");
}

void JoyTopic::SubJoyBtn(const Joy::SharedPtr joy)
{
    if(joy->buttons[2] == 1){
        buttonOnOff.x_btn = true;
    }else{
        buttonOnOff.x_btn = false;
    }

    if(joy->buttons[5] == 1){
        buttonOnOff.rb_btn = true;
    }else{
        buttonOnOff.rb_btn = false;
    }
}

void JoyTopic::LoggerBtnText(const Joy::SharedPtr joy)
{
    if(joy->buttons[0] == 1){
        RCLCPP_INFO(this->get_logger(), "A Button");
    }
    if(joy->buttons[1] == 1){
        RCLCPP_INFO(this->get_logger(), "B Button");
    }
    if(joy->buttons[2] == 2){
        RCLCPP_INFO(this->get_logger(), "X Button");
    }
    if(joy->buttons[3] == 1){
        RCLCPP_INFO(this->get_logger(), "Y Button");
    }
    if(joy->buttons[4] == 1){
        RCLCPP_INFO(this->get_logger(), "LB Button");
    }
    if(joy->buttons[5] == 1){
        RCLCPP_INFO(this->get_logger(), "RB Button");
    }
    if(joy->buttons[6] == 1){
        RCLCPP_INFO(this->get_logger(), "BACK Button");
    }
    if(joy->buttons[7] == 1){
        RCLCPP_INFO(this->get_logger(), "START Button");
    }
    if(joy->buttons[8] == 1){
        RCLCPP_INFO(this->get_logger(), "MAIN Button");
    }
    if(joy->buttons[9] == 1){
        RCLCPP_INFO(this->get_logger(), "LJC Button");
    }
    if(joy->buttons[10] == 1){
        RCLCPP_INFO(this->get_logger(), "RJC Button");
    }
    if(joy->buttons[11] == 1){
        RCLCPP_INFO(this->get_logger(), "LL Button");
    }
    if(joy->buttons[12] == 1){
        RCLCPP_INFO(this->get_logger(), "LR Button");
    }
    if(joy->buttons[13] == 1){
        RCLCPP_INFO(this->get_logger(), "LU Button");
    }
    if(joy->buttons[14] == 1){
        RCLCPP_INFO(this->get_logger(), "LD Button");
    }
}

void JoyTopic::LoggerAxesText(const Joy::SharedPtr joy)
{
    if(joy->axes[0] != 0){
        if(joy->axes[0] < 0){
            RCLCPP_INFO(this->get_logger(), "L right %f", joy->axes[0]);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "L left %f", joy->axes[0]);
        }
    }
    if(joy->axes[1] != 0){
        if(joy->axes[1] < 0){
            RCLCPP_INFO(this->get_logger(), "L down %f", joy->axes[1]);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "L up %f", joy->axes[1]);
        }
    }

    if(joy->axes[2] < 0.9f){
        RCLCPP_INFO(this->get_logger(), "LT %f", joy->axes[2]);
    }

    if(joy->axes[3] != 0){
        if(joy->axes[3] < 0){
            RCLCPP_INFO(this->get_logger(), "R right %f", joy->axes[3]);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "R left %f", joy->axes[3]);
        }
    }
    if(joy->axes[4] != 0){
        if(joy->axes[4] < 0){
            RCLCPP_INFO(this->get_logger(), "R right %f", joy->axes[4]);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "R left %f", joy->axes[4]);
        }
    }

    if(joy->axes[5] < 0.9f){
        RCLCPP_INFO(this->get_logger(), "LT %f", joy->axes[2]);
    }

    if(joy->axes[6] != 0){
        if(joy->axes[6] == 1.0f){
            RCLCPP_INFO(this->get_logger(), "LB %.1f", joy->axes[6]);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "LB %.1f", joy->axes[6]);
        }
    }
    if(joy->axes[7] != 0){
        if(joy->axes[7] == 1.0f){
            RCLCPP_INFO(this->get_logger(), "LB %.1f", joy->axes[7]);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "LB %.1f", joy->axes[7]);
        }
    }
}


void JoyTopic::SubTwistJoy(const Twist::SharedPtr joyt)
{
    // x  = -0.699999 ~ 0.699999
    // rb = -1.499999 ~ 1.499999

    // z  = -0.399999 ~ 0.399999
    // rb = -0.999999 ~ 0.999999

    // RCLCPP_INFO(this->get_logger(), "\nlinear.x = %f\nangular.z = %f", joyt->linear.x, joyt->angular.z);
    fw_msg.control_mode = 4;

    // 전진
    // x  = -0.699999 ~ 0.699999
    if(buttonOnOff.x_btn == true && joyt->linear.x > highSpeed){
        fw_msg.command_1 = highSpeed; // 모빌리티
        tw_msg.linear.x = highSpeed; // 터틀봇
    }
    else if(buttonOnOff.x_btn == true && joyt->linear.x < lowSpeed)
    {
        fw_msg.command_1 = lowSpeed;
        tw_msg.linear.x = lowSpeed;
    }
    else if(buttonOnOff.rb_btn == true && joyt->linear.x > powerHighSpeed){
        fw_msg.command_1 = powerHighSpeed;
        tw_msg.linear.x = powerHighSpeed;
    }
    else if(buttonOnOff.rb_btn == true && joyt->linear.x < powerLowSpeed){
        fw_msg.command_1 = powerLowSpeed;
        tw_msg.linear.x = powerLowSpeed;
    }
    else{
        fw_msg.command_1 = joyt->linear.x;
        tw_msg.linear.x = joyt->linear.x;
        tw_msg.linear.y = joyt->linear.y;
        tw_msg.linear.z = joyt->linear.z;
    }

    // 회전
    if(buttonOnOff.x_btn == true && joyt->angular.z > highTurn){
        fw_msg.command_2 = highTurn;
        tw_msg.angular.z = highTurn;
    }
    else if(buttonOnOff.x_btn == true && joyt->angular.z < lowTurn){
        fw_msg.command_2 = lowTurn;
        tw_msg.angular.z = lowTurn;
    }
    else if(buttonOnOff.rb_btn == true && joyt->angular.z > powerHighTurn){
        fw_msg.command_2 = powerHighTurn;
        tw_msg.angular.z = powerHighTurn;
    }
    else if(buttonOnOff.rb_btn == true && joyt->angular.z < powerLowTurn){
        fw_msg.command_2 = powerLowTurn;
        tw_msg.angular.z = powerLowTurn;
    }
    else{
        fw_msg.command_2 = joyt->angular.z;
        tw_msg.angular.x = joyt->angular.x;
        tw_msg.angular.y = joyt->angular.y;
        tw_msg.angular.z = joyt->angular.z;
    }
    RCLCPP_INFO(this->get_logger(), "\n------------\nlinear.x = %f\nangular.z = %f", fw_msg.command_1, fw_msg.command_2);
    // turtlebot3Pub_->publish(tw_msg);
    cmd_pub_->publish(fw_msg);
}

void JoyTopic::WebJoyTopic(const std_msgs::msg::Float64MultiArray::SharedPtr web){
  RCLCPP_INFO(this->get_logger(), "[0]:%lf ---- [1]:%lf", web->data[0], web->data[1]);
  RCLCPP_INFO(this->get_logger(), "----------------------------");

  if(buttonOnOff.x_btn == true || buttonOnOff.rb_btn == true){
      return;
  }
  // fw_msg.control_mode = 4;

  double y = Mapping(web->data[0], -1.0, 1.0, lowTurn, highTurn);
  double x = Mapping(web->data[1], -1.0, 1.0, lowSpeed, highSpeed);
  RCLCPP_INFO(this->get_logger(), "2::%lf, %lf", x, y);

  fw_msg.command_1 = x;
  fw_msg.command_2 = y;

  cmd_pub_->publish(fw_msg);
}


double JoyTopic::Mapping(double x, double in_min, double in_max, double out_min, double out_max){
    if (x == 0) {
        return 0;
    }

    const double scale_pos = out_max / in_max;
    const double scale_neg = out_min / in_min;

    if (x > 0) {
        return x * scale_pos;
    }
    else {
        return x * scale_neg;
    }

    // if(x == 0){
    //     return 0;
    // }
    // if((in_max - in_min) > (out_max - out_min)){
    //     return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
    // }else{
    //     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // }
}

// 긴급 정지
void JoyTopic::WebStopBtnCB(const std_msgs::msg::String::SharedPtr btn){
    RCLCPP_INFO(this->get_logger(), "on button %s", btn);
}

/**
 *
 */

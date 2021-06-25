# control_mode : 1 - wheel move distance, 2 - wheel move speed, 3 - wheel move point, 4 - robot move speed
# command_1 : wheel move distance - left distance(m), wheel move speed - left wheel velocity(m/s),
# wheel move point - x point, robot move speed - linear velocity(m/s)
# command_2 : wheel move distance - right distance(m), wheel move speed - right wheel velocity(m/s),
# wheel move point - y point, robot move speed - angular velocity(radian/s)
# std_msgs/Header header
uint8 control_mode
float64 command_1
float64 command_2

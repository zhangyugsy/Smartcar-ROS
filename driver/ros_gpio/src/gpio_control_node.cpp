#include "ros/ros.h"
#include "ros_gpio_control/gpio.h"
#include "std_msgs/Bool.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_single");
    int gpio_number;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("gpio_number", gpio_number, 389);

    GPIO ultrasonic_single(gpio_number);
    ultrasonic_single.edge(GPIO::EDGE_BOTH);

    ros::Publisher hall_sensor_pub = nh.advertise<std_msgs::Bool>("hall_sensor", 10);
    ros::Rate loop_rate(60);

    while (ros::ok()) {
        ultrasonic_single.poll(100);
        std_msgs::Bool msg;
        msg.data = ultrasonic_single;
        hall_sensor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

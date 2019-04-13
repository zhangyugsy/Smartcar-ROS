#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <smartcar_msgs/DiffSonic.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

double A = 1.00595663;
double B = 149.61707504;

double calibrated(int origin)
{
    return A * origin + B;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_serial_port");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ros::Publisher pub_left = nh.advertise<std_msgs::Float32>("/sonic_left", 1);
    // ros::Publisher pub_right = nh.advertise<std_msgs::Float32>("/sonic_right", 1);
    // std_msgs::Float32 sonic_left, sonic_right;

    std::string port_left, port_right;
    pnh.param<std::string>("port_left", port_left, "/dev/ttyUSB0");
    pnh.param<std::string>("port_right", port_right, "/dev/ttyUSB1");

    ros::Publisher pub_sonic_raw_ = nh.advertise<smartcar_msgs::DiffSonic>("/sonic_raw", 1);
    ros::Publisher pub_sonic_filtered_ = nh.advertise<smartcar_msgs::DiffSonic>("/sonic_filtered", 1);
    smartcar_msgs::DiffSonic msg_sonic;

    serial::Serial sp_left;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp_left.setPort(port_left);
    sp_left.setBaudrate(115200);
    sp_left.setTimeout(to);

    serial::Serial sp_right;
    sp_right.setPort(port_right);
    sp_right.setBaudrate(115200);
    sp_right.setTimeout(to);

    try {
        sp_left.open();
        sp_right.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    ros::Rate loop_rate(50);

    int count_left = 0;
    int count_right = 0;

    while (ros::ok()) {
        size_t n_left = sp_left.available();
        size_t n_right = sp_right.available();

        if (n_left != 0) {
            uint8_t buffer[1024];
            n_left = sp_left.read(buffer, n_left);
            // for (int i = 0; i < n_left; i++) {
            //     //16进制的方式打印到屏幕
            //     std::cout << std::hex << (buffer[i] & 0xff) << " ";
            // }
            if (n_left < 3 || (buffer[0] & 0xff) != 165) {
                count_left++;
                continue;
            } else {
                int a = buffer[1] & 0xff;
                int b = buffer[2] & 0xff;
                int left_origin = a * 256 + b;
                // sonic_left.data = calibrated(left_origin);
                // pub_left.publish(sonic_left);
                msg_sonic.left = calibrated(left_origin);
                count_left = 0;
            }
            // std::cout << " left: " << msg_sonic.left << std::endl;
        } else {
            count_left++;
        }

        if (n_right != 0) {
            uint8_t buffer[1024];
            n_right = sp_right.read(buffer, n_right);
            if (n_right < 3 || (buffer[0] & 0xff) != 165) {
                count_right++;
                continue;
            } else {
                int a = buffer[1] & 0xff;
                int b = buffer[2] & 0xff;
                int right_origin = a * 256 + b;
                // sonic_right.data = calibrated(right_origin);
                // pub_right.publish(sonic_right);
                msg_sonic.right = calibrated(right_origin);
                count_right = 0;
            }
            // std::cout << "right: " << msg_sonic.right << std::endl;
        } else {
            count_right++;
        }
        if (count_left > 10 || count_right > 10) {
            if (count_left > 10) {
                msg_sonic.type = "sonic_left_fail";
                ROS_WARN_STREAM("sonic_left_fail");
            } else {
                msg_sonic.type = "sonic_right_fail";
                ROS_WARN_STREAM("sonic_right_fail");
            }
            msg_sonic.left = 0.0;
            msg_sonic.right = 0.0;
            pub_sonic_raw_.publish(msg_sonic);
        } else {
            msg_sonic.type = "sonic_good";
            pub_sonic_raw_.publish(msg_sonic);
        }
        loop_rate.sleep();
    }

    //关闭串口
    sp_left.close();
    sp_right.close();

    return 0;
}

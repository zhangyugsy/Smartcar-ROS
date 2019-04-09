
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <vector>

serial::Serial ser; //声明串口对象
// kalman param
double Q_Covariance;
double R_Covariance;
double pre_P_Covariance, cur_P_Covariance;
double filtered_dis, pre_dis, pre_pre_dis;
double K;

void linearKalmanFilter(const double& measure_value, const double& pre_value, const double& pre_P_Covariance,
    double& cur_value, double& cur_P_Covariance)
{
    /**
     * pre_speed是上一时刻的预测值  
     * pre_P_Covariance是上一时刻的预测值误差的协方差矩阵 
     * tmp_P_Covariance是这一时刻测量值误差的协方差矩阵的中间值 P_t|t-1
     * cur_P_Covariance是这一时刻测量值误差的协方差矩阵
     * Q_Covariance 是预测值的高斯噪声的协方差矩阵
     * K=卡尔曼增益
     * 本实例H=1 F=1
     * 符号定义参考：https://zhuanlan.zhihu.com/p/36745755
    */

    double tmp_P_Covariance = pre_P_Covariance + Q_Covariance;
    K = tmp_P_Covariance / (tmp_P_Covariance + R_Covariance);
    cur_value = pre_value + K * (measure_value - pre_value);
    cur_P_Covariance = tmp_P_Covariance - K * tmp_P_Covariance;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "uart_ros");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher read_pub = nh.advertise<std_msgs::Float32>("ultrasonic_dis", 1000);
    nh_private.param("Q_Covariance", Q_Covariance, double(1.5));
    nh_private.param("R_Covariance", R_Covariance, double(1.0));
    pre_P_Covariance = Q_Covariance / 10.;
    pre_dis = 10;

    try {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyTHS2");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (ser.available()) {
            //ROS_INFO_STREAM("Reading from serial port");
            std_msgs::UInt8MultiArray serial_data;
            std_msgs::Float32 dis_msg;

            ser.read(serial_data.data, ser.available());

            double raw_dis = (float(serial_data.data[1]) * 256 + float(serial_data.data[2])) * 1.0 / 1000.;
            if (raw_dis > 8) {
                continue;
            }

            linearKalmanFilter(raw_dis, pre_dis, pre_P_Covariance, filtered_dis, cur_P_Covariance);

            pre_dis = filtered_dis;
            pre_P_Covariance = cur_P_Covariance;

            dis_msg.data = filtered_dis;
            read_pub.publish(dis_msg);
            //char tmp;
            //for(size_t i = 0; i < serial_data.data.size(); i++){
            //    tmp = serial_data.data[i];
            //    std::cout << int(tmp) << std::endl;
            //}
        }
        loop_rate.sleep();
    }
}


#include <ros/ros.h> 
#include <ros/time.h>
#include <serial/serial.h> 
#include <std_msgs/Float32.h> 
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Empty.h> 
#include <sstream>
#include <iostream>
#include <vector>
 
serial::Serial ser; //声明串口对象
 
int main (int argc, char** argv) 
{ 

    ros::init(argc, argv, "uart_ros"); 

    ros::NodeHandle nh; 
  
    ros::Publisher read_pub = nh.advertise<std_msgs::Float32>("ultrasonic_dis", 1000);
  
try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyTHS2"); 
        ser.setBaudrate(9600); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
 
 if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
 
 ros::Rate loop_rate(10); 
    while(ros::ok()) 
    { 
        if(ser.available()){ 
            //ROS_INFO_STREAM("Reading from serial port"); 
            std_msgs::UInt8MultiArray serial_data;
            std_msgs::Float32 dis;

            ser.read(serial_data.data, ser.available());
            
            dis.data = (float(serial_data.data[1]) * 256 + float(serial_data.data[2])) * 1.0 / 1000.;
            
            //char tmp;
            //for(size_t i = 0; i < serial_data.data.size(); i++){
            //    tmp = serial_data.data[i];
            //    std::cout << int(tmp) << std::endl;
            //}
            
            read_pub.publish(dis); 

        } 
 
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
}


#include "jetsonGPIO.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
using namespace std;

int getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO | ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ultrasonic-single");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher pub_ultra_single;
    pub_ultra_single = nh.advertise<std_msgs::Float32>("ultrasonic_dis", 10);
    std_msgs::Float32 dis_msg;

    cout << "Testing the GPIO Pins" << endl;

    jetsonTX2GPIONumber echo = gpio255; // Input
    jetsonTX2GPIONumber trig = gpio397; // Output

    gpioExport(echo);
    gpioExport(trig);

    gpioSetDirection(echo, inputPin);
    gpioSetDirection(trig, outputPin);

    // Wait for the push button to be pressed
    cout << "Press the ESC key to quit the program" << endl;

    while (getkey() != 27) {
        printf("------------------------");
        unsigned int value = 1;

        unsigned int pre_value = 1;

        unsigned int rising_flag = 0;
        unsigned int falling_flag = 0;
        unsigned int high_flag = 0;

        int pre_pre_dis = 0;
        int pre_dis = 0;
        int dis = 0;
        int dis_count = 0;

        gpioSetValue(trig, high);
        usleep(11);
        gpioSetValue(trig, low);

        while (true) {

            gpioGetValue(echo, &value);
            cout << "value: " << value << endl;
            dis_msg.data = float(value);
            pub_ultra_single.publish(dis_msg);

            if (value == 1) {
                dis_count++;
                high_flag = 1;
            }

            if (high_flag == 1) {
                if (value == 0)
                    falling_flag = 1;
            }

            /*
            if (value == 1 && pre_value == 0){
                rising_flag = 1;
                falling_flag = 0;
            }
            
            if (value == 0 && pre_value == 1){
                falling_flag = 1;
                rising_flag = 0;
            }
            */

            pre_value = value;
            //if (rising_flag == 1){
            //    dis_count++;
            //}

            // TODO linear kalman
            if (falling_flag == 1) {
                cout << "dis: " << dis_count << endl;
                dis_count = 0;
                falling_flag = 0;
                break;
                //dis = dis_count * 0.5 + pre_dis * 0.3 +pre_pre_dis * 0.2;
                //pre_pre_dis = pre_dis;
                //pre_dis = dis;

                // cout << "dis value: " << dis << endl;
                //dis_count = 0;
                //falling_flag = 0;
                //rising_flag = 0;
                //break;
            }
        }

        usleep(1000000);
    }

    cout << "GPIO example finished." << endl;
    gpioUnexport(echo); // unexport the pin
    gpioUnexport(trig); // unexport the pin
    return 0;
}

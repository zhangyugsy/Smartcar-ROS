#include "jetsonGPIO.h"
#include <iostream>
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

    cout << "Testing the GPIO Pins" << endl;

    jetsonTX2GPIONumber echo = gpio389; // Input

    gpioExport(echo);

    gpioSetDirection(echo, inputPin);

    // Wait for the push button to be pressed
    cout << "Please press the button! ESC key quits the program" << endl;

    unsigned int value = 1;

    unsigned int pre_value = 1;

    unsigned int rising_flag = 0;
    unsigned int falling_flag = 0;

    int pre_pre_dis = 0;
    int pre_dis = 0;
    int dis = 0;
    int dis_count = 0;

    while (getkey() != 27) {
        gpioGetValue(echo, &value);

        if (value == 1 && pre_value == 0) {
            rising_flag = 1;
            falling_flag = 0;
        }

        if (value == 0 && pre_value == 1) {
            falling_flag = 1;
            rising_flag = 0;
        }

        if (rising_flag == 1) {
            dis_count++;
        }

        // TODO linear kalman
        if (falling_flag == 1) {

            dis = dis_count * 0.5 + pre_dis * 0.3 + pre_pre_dis * 0.2;
            pre_pre_dis = pre_dis;
            pre_dis = dis;

            cout << "dis value: " << dis << endl;
            dis_count = 0;
            falling_flag = 0;
            rising_flag = 0;
        }

        pre_value = value;
    }

    cout << "GPIO example finished." << endl;
    gpioUnexport(echo); // unexport the push button
    return 0;
}

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <controller/Rover.h>

// C library headers
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <bitset>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define LEFT_DRIVE 0
#define RIGHT_DRIVE 1
#define AUGER 2
#define DUMP 3
#define VERIFICATION 7

enum AXIS {
  LEFT_STICK_LR,
  LEFT_STick_UD,
  LT,
  RIGHT_STICK_LR,
  RIGHT_STICK_UD,
  RT,
  D_PAD_LR,
  D_PAD_UD
};

enum BUTTONS {
  A,
  B,
  X, 
  Y,
  LB,
  RB,
  SELECT,
  START,
  XBOX,
  LEFT_STICK,
  RIGHT_STICK
};

class Serial{
  public:
    Serial(char* path);

  private:
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
    int initTTY();
    void send_package();
    int wait_for_reply();
    void publish_rover_package(uint8_t (&buf)[8]);

    ros::NodeHandle nh_;

    ros::Publisher m_rov_pub_;
    ros::Subscriber m_joy_sub_;

    uint8_t m_package[8] = {0,0,0,0,0,0,0,0};

    int m_serial_port;
    struct termios m_tty;
};

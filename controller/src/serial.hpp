// ROS Includes
#include <ros/ros.h>
#include <controller/Rover.h>
#include "joystick.hpp"
#include <controller/IR_Data.h>

// C library headers
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <bitset>
#include <chrono>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define LEFT_DRIVE 0
#define RIGHT_DRIVE 1
#define RAIL 2
#define AUGER_DRIVE 3
#define DUMP 4
#define VERIFICATION 7

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

enum AXIS {
  LEFT_STICK_LR,
  LEFT_STICK_UD,  // Left side drive
  LT,             // Left Auger dig
  RIGHT_STICK_LR,
  RIGHT_STICK_UD, // Right side drive
  RT,             // Right Auger dig
  D_PAD_LR,       // Rail movement
  D_PAD_UD        // Rail Rotation
};

enum BUTTONS {
  A,
  B,              // Dump Routine
  X, 
  Y,
  LB,             // Left Auger Reverse
  RB,             // Right Auger Reverse
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
    void joystickCallback(const controller::JoyCon::ConstPtr& joy);
    void irCallback(const controller::IR_Data::ConstPtr& msg);
    int initTTY();
    void send_package(int count);
    int wait_for_reply(int& count);
    void publish_rover_package(uint8_t (&buf)[8]);

    ros::NodeHandle nh_;

    ros::Publisher m_rov_pub_;
    ros::Subscriber m_joy_sub_;
    ros::Subscriber m_ir_sub_;

    uint8_t m_package[8] = {0,0,0,0,0,0,0,0};

    int m_serial_port;
    struct termios m_tty;

    uint8_t m_auger_speed;
    bool using_ir_sensors = false;
};


class Rover{

public:
    Rover();
    
private:

    void messageCallback(const controller::Rover::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber m_rover_sub_;

    uint8_t m_package[8] = {0,0,0,0,0,0,0,0};
};


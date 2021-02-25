// ROS Includes
#include <ros/ros.h>
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

struct serial_package {
    uint8_t right_vel;
    uint8_t left_vel;
    uint8_t reserved[5];
    uint8_t verification;
};

int initTTY(int serial_port, termios& tty){
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;
}


class Serial{
  public:
    Serial(char* path);

  private:
    void sendDataCallback(const controller::Rover::ConstPtr& msg);
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    ros::Publisher m_rov_pub_;
    ros::Subscriber m_rov_sub_;


    int m_serial_port;
    struct termios m_tty;
};


void Serial::sendDataCallback(const controller::Rover::ConstPtr& msg){

    serial_package buf;

    uint8_t buff[8];

    buff[0] = msg->right_vel;
    buff[1] = msg->left_vel;
    buff[2] = msg->auger;
    buff[3] = msg->dump;
    buff[4] = msg->reserved;
    buff[5] = msg->reserved1;
    buff[6] = msg->reserved8;
    buff[7] = msg->verification;

    std::cout << buff << std::endl;

    write(m_serial_port, &buff, sizeof(buff));
    char recv[8];
    int num_bytes;
    int count = 0;
    while(count < 8){
      num_bytes = 0;
      num_bytes = read(m_serial_port, &recv, sizeof(recv));
      count += num_bytes;
    }
    for(int i=0; i<sizeof(recv);i++){
      std::cout << std::bitset<8>(recv[i]).to_string() << std::endl;
    }
}


Serial::Serial(char* path){
    m_serial_port = open(path, O_RDWR);

    initTTY(m_serial_port, m_tty);

    cfsetispeed(&m_tty, B19200);
    cfsetospeed(&m_tty, B19200);

    // Save tty settings, also checking for error
    if (tcsetattr(m_serial_port, TCSANOW, &m_tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    m_rov_pub_ = nh_.advertise<controller::Rover>("rover", 1);


    m_rov_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Serial::joystickCallback, this);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fpga_stream");
  Serial serial = Serial(argv[1]);

  ros::spin();
}

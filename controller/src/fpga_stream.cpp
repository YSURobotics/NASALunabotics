#include "serial.hpp"

//Initialize the serial port
int Serial::initTTY(){
  if(tcgetattr(m_serial_port, &m_tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  m_tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  m_tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  m_tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  m_tty.c_cflag |= CS8; // 8 bits per byte (most common
  m_tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  m_tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  m_tty.c_lflag &= ~ICANON;
  m_tty.c_lflag &= ~ECHO; // Disable echo
  m_tty.c_lflag &= ~ECHOE; // Disable erasure
  m_tty.c_lflag &= ~ECHONL; // Disable new-line echo
  m_tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  m_tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  m_tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  m_tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  m_tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  m_tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  m_tty.c_cc[VMIN] = 0;
}


Serial::Serial(char* path){

  m_serial_port = open(path, O_RDWR);

  initTTY();

  //FPGA uses 19200
  cfsetispeed(&m_tty, B19200);
  cfsetospeed(&m_tty, B19200);

  //Save tty settings, also checking for error
  if (tcsetattr(m_serial_port, TCSANOW, &m_tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  //Init 'rover' topic to be published
  m_rov_pub_ = nh_.advertise<controller::Rover>("rover", 1);

  //Setting up to listen to the controller node
  m_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Serial::joystickCallback, this);
}


void Serial::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy){

  //Place all the controller inputs in the package buffer
  m_package[LEFT_DRIVE] = (joy->axes[AXIS::LEFT_STick_UD] * 10) + 10;
  m_package[RIGHT_DRIVE] = (joy->axes[AXIS::RIGHT_STICK_UD] * 10) + 10;
  m_package[AUGER] = joy->buttons[BUTTONS::A];
  m_package[DUMP] = joy->buttons[BUTTONS::B];

  //Send package to FPGA
  send_package();
}

void Serial::send_package() {

  //Get a random verification byte
  m_package[VERIFICATION] = random();

  //Actually write buffer to USB
  write(m_serial_port, &m_package, sizeof(m_package));

  //Make sure package is received and interpreted correctly
  if(wait_for_reply())
    send_package();
}

int Serial::wait_for_reply(){
  //temp buf
  uint8_t recv[8];

  //make sure to read in 8 bytes
  int count = 0;
  while(count < 8){
    count += read(m_serial_port, &recv, sizeof(recv) - count);
  }

  //Make sure the message was verified, if not, resend the same package
  if(recv[VERIFICATION] != m_package[VERIFICATION] + 1){
    return 1;
  }

  publish_rover_package(recv);

}

void Serial::publish_rover_package(uint8_t (&buf)[8]){
  //Create temp rosmsg object
  controller::Rover msg;

  //Place values in msg
  msg.left_vel = buf[LEFT_DRIVE];
  msg.right_vel = buf[RIGHT_DRIVE];
  msg.auger = buf[AUGER];
  msg.dump = buf[DUMP];
  msg.reserved = buf[4];
  msg.reserved1 = buf[5];
  msg.reserved8 = buf[6];
  msg.verification = buf[VERIFICATION];

  //publish msg to rover topic
  m_rov_pub_.publish(msg);
}


int main(int argc, char** argv)
{

  if(argc > 2){
    std::cerr << "Too many arguments!! Please only include USB Device path!" << std::endl;
    return 1;
  }
  if(argc < 2){
    std::cerr << "Too few arguments!! Please include USB Device path! \n" <<
    "command should look like: rosrun controller fpga_stream [path of usb device] \n" <<
    "ex. rosrun controller fpga_stream /dev/ttyUSB1" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "fpga_stream");
  //argv[1] should be path to usb interface
  Serial serial = Serial(argv[1]);

  ros::spin();
}

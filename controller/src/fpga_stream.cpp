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

  std::cout << path << std::endl;
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
  m_joy_sub_ = nh_.subscribe<controller::JoyCon>("joycon", 1, &Serial::joystickCallback, this);

  //Setting up to listen to ir data
  m_ir_sub_ = nh_.subscribe<controller::IR_Data>("ir_array", 1, &Serial::irCallback, this);


  m_auger_speed = 0;
}

void Serial::irCallback(const controller::IR_Data::ConstPtr& msg){
  if(!using_ir_sensors){
    return;
  }
  m_package[LEFT_DRIVE]  = 11;
  m_package[RIGHT_DRIVE] = 11;
  m_package[AUGER_DRIVE] = 0;
  m_package[RAIL]        = 0;
  m_package[DUMP]        = 0;

  double angle = msg->direction;

  if(angle == 0){
    m_package[RIGHT_DRIVE] = 21; 
    m_package[LEFT_DRIVE] = 21;
  } else if(angle == 90){
    m_package[RIGHT_DRIVE] = 1; 
    m_package[LEFT_DRIVE] = 21;
  } else if(angle == 180){
    m_package[RIGHT_DRIVE] = 1; 
    m_package[LEFT_DRIVE] = 1;
  } else if(angle == 270) {
    m_package[RIGHT_DRIVE] = 21; 
    m_package[LEFT_DRIVE] = 1;
  } else if(angle > 0 && angle < 90){
    m_package[RIGHT_DRIVE] = 21 - (21 *  (angle / 90)); 
    m_package[LEFT_DRIVE] = 21;
  } else if ( angle > 90 && angle < 180){
    m_package[RIGHT_DRIVE] = 1; 
    m_package[LEFT_DRIVE] = 21;
  } else if ( angle > 180 && angle < 270){
    m_package[RIGHT_DRIVE] = 21;
    m_package[LEFT_DRIVE] = 1;
  } else if ( angle > 270 && angle < 360){
    m_package[RIGHT_DRIVE] = 21;
    m_package[LEFT_DRIVE] = 21 - (21 *  ((angle - 270) / 90));
  }

  send_package: 
    send_package(0);
}

void Serial::joystickCallback(const controller::JoyCon::ConstPtr& joy){

    //Place all the controller inputs in the package buffer
    if(!using_ir_sensors){
      m_package[LEFT_DRIVE]  = (joy->LS_UD / (joycon::AXIS_RANGE / 10)) + 11;
      //(m_axes[LS_UD] / (AXIS_RANGE / 11) + 11
      m_package[RIGHT_DRIVE] = (joy->RS_UD / (joycon::AXIS_RANGE / 10)) + 11;
    } else {
       m_package[LEFT_DRIVE] =  m_package[LEFT_DRIVE];
       m_package[RIGHT_DRIVE] = m_package[RIGHT_DRIVE];
    }
    m_package[AUGER_DRIVE] = 0;
    m_package[RAIL]        = 0;
    m_package[DUMP]        = 0;


    if(joy->Y)
      m_auger_speed ^= BIT4;    // Toggle speed

    if(joy->SELECT)
      using_ir_sensors = !using_ir_sensors; // Toggle IR Sensors usage


  dump: 
    if((joy->X == 0 && joy->B == 0) ||
       (joy->X != 0 && joy->B != 0)) {
      goto rail;
    }

    if(joy->X){      // Lower Dump
      m_package[RAIL] |= BIT0;
    }
    else if(joy->B){ // Tilt dump
      m_package[RAIL] |= BIT1;
    }
    

  rail:
    if((joy->DPAD_UD == 0 && joy->DPAD_LR == 0) ||
       (joy->DPAD_UD != 0 && joy->DPAD_LR != 0)){
      goto auger_drive;
    }

    if(joy->DPAD_LR < 0) // RIGHT
      m_package[RAIL] |= BIT0;
    else if(joy->DPAD_LR > 0) // LEFT
      m_package[RAIL] |= BIT1;
    
    if(joy->DPAD_UD > 0) // UP
      m_package[RAIL] |= BIT2;
    else if(joy->DPAD_UD < 0) // DOWN
      m_package[RAIL] |= BIT3;      


    m_package[RAIL] |= m_auger_speed;

  auger_drive:
    if(joy->LT < 0 && joy->RT < 0  &&
       joy->LB == 0 && joy->RB == 0)
      {
        goto send_package;
      }

    if(joy->RT > 0 && joy->RB) {} // If both on, do nothing
    else if(joy->RT > 0)          // RIGHT DRIVE
      m_package[AUGER_DRIVE] |= BIT0;
    else if(joy->RB)    // RIGHT BACK
      m_package[AUGER_DRIVE] |= BIT1;

    if(joy->LT > 0 && joy->LB) {} // If both on, do nothing
    else if(joy->LT > 0)           // LEFT DRIVE
      m_package[AUGER_DRIVE] |= BIT2;
    else if(joy->LB)     // LEFT BACK
      m_package[AUGER_DRIVE] |= BIT3;


  send_package:
    //Send package to FPGA
    send_package(0);
}

void Serial::send_package(int count) {

  //Get a random verification byte
  m_package[VERIFICATION] = random() % 255; // Using mod here so we dont need to deal with overflow when adding one

  //Actually write buffer to USB
  write(m_serial_port, &m_package, sizeof(m_package));

  //Make sure package is received and interpreted correctly
  if(wait_for_reply(count))
    send_package(count);
}

int Serial::wait_for_reply(int& sent){
  //temp buf
  uint8_t recv[8];

  //make sure to read in 8 bytes
  int count = 0;
  while(count < 8){
    count += read(m_serial_port, &recv, sizeof(recv) - count);
  }

  //Make sure the message was verified, if not, resend the same package
  if(recv[VERIFICATION] != m_package[VERIFICATION]){
    if(sent < 2){
      printf("\x1B[33mSlow Down! Could not verify packet! Resent %d times\033[0m\n", sent);
    }
    else {
      printf("\x1B[31mSlow Down! Could not verify packet! Resent %d times\033[0m\n", sent);
    }
    sent++;
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
  msg.rail = buf[RAIL];
  msg.auger = buf[AUGER_DRIVE];
  msg.dump = buf[DUMP];
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
    "Command should look like: rosrun controller fpga_stream [path of usb device] \n" <<
    "ex. rosrun controller fpga_stream /dev/ttyUSB1" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "fpga_stream");
  //argv[1] should be path to usb interface
  Serial serial = Serial(argv[1]);

  ros::spin();
}

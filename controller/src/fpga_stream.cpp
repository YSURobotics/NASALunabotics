#include "serial.hpp"

/*
 * To use this you call rosrun controller fpga_stream <path to FPGA>
 *
 * To check the path of the FPGA run 'ls /dev' in a terminal and look for 'ttyUSBX'
 * change 'X' appropriately
 */

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

  /*
   * This is your pointer to that '/dev/ttyUSB' path. Linux treats these devices like files
   *
   * Think of this as a file that you will write to and read to like any other file and the attached device will do the same
   */
  m_serial_port = open(path, O_RDWR);

  // This calls that mangled mess above. No, you do not need to know all of those flags
  initTTY();

  //FPGA uses 19200 baud rate
  cfsetispeed(&m_tty, B19200);
  cfsetospeed(&m_tty, B19200);

  //Save tty settings, also checking for error
  if (tcsetattr(m_serial_port, TCSANOW, &m_tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  /*
   * Init rover packet publisher
   * @1st param: tells what the topic name should be used for ehat we are publishing
   * @2nd param: sets the queue size that will be held for this topic, we are using 1 to ensure we always get the newest packet
   */
  m_rov_pub_ = nh_.advertise<controller::Rover>("rover", 1);

  /*Setting up to listen to the controller and IR nodes respectively
   * @1st param: name of the topic we want to listen to updates on
   * @2nd param: sets the queue size that we want to maintain here, using 1 for newest packet
   * @3rd param: location of the callback function that will be called when there is an update
   * @4th param: what we want to pass as context or parameters to the callback function this will pass the object we are in right now
   */
  m_joy_sub_ = nh_.subscribe<controller::JoyCon>("joycon", 1, &Serial::joystickCallback, this);
  m_ir_sub_ = nh_.subscribe<controller::IR_Data>("ir_array", 1, &Serial::irCallback, this);
}

/*
 * A function to handle the data from the IR sensors
 */
void Serial::irCallback(const controller::IR_Data::ConstPtr& msg){
  //If the flag to use the sensors is not set, skip this
  if(!using_ir_sensors){
    return;
  }

  //Set everything to neutral values
  m_package[LEFT_DRIVE]  = 0;
  m_package[RIGHT_DRIVE] = 0;
  m_package[AUGER_DRIVE] = 0;
  m_package[RAIL]        = 0;
  m_package[DUMP]        = 0;

  //Get the angle from the IR msg. THIS IS IN DEGREES
  double angle = msg->direction;


  if(angle == 0){   // If the sensor is straight ahead we set to full throttle ahead
    m_package[RIGHT_DRIVE] = 21; 
    m_package[LEFT_DRIVE] = 21;
  } else if(angle == 90){ // If the sensor is directly to the right we should go full forward on left, full reverse on right
    m_package[RIGHT_DRIVE] = 1; 
    m_package[LEFT_DRIVE] = 21;
  } else if(angle == 180){ // If it is perfectly behind us reverse
    m_package[RIGHT_DRIVE] = 1; 
    m_package[LEFT_DRIVE] = 1;
  } else if(angle == 270) { // If the sensor is directly to the left we should go full forward on right, full reverse on left
    m_package[RIGHT_DRIVE] = 21; 
    m_package[LEFT_DRIVE] = 1;
  } else if(angle > 0 && angle < 90){ //Scale the turn if it is in the first quadrant
    m_package[RIGHT_DRIVE] = 21 - (21 *  (angle / 90)); 
    m_package[LEFT_DRIVE] = 21;
  } else if ( angle > 270 && angle < 360){ // Scale turn if it is in the fourth quadrant
      m_package[RIGHT_DRIVE] = 21;
      m_package[LEFT_DRIVE] = 21 - (21 *  ((angle - 270) / 90));


      /* if it is slightly behind us, do a donut until its not, avoid reverse for stress on gearbox */
  } else if ( angle > 90 && angle < 180){
    m_package[RIGHT_DRIVE] = 1; 
    m_package[LEFT_DRIVE] = 21;
  } else if ( angle > 180 && angle < 270) {
      m_package[RIGHT_DRIVE] = 21;
      m_package[LEFT_DRIVE] = 1;
  }

  // Call the function to send a packet and let it know it is the first try with this packet
  send_package(0);
}


/*
 * This is a callback function that is called when the "joycon" topic is updated
 *
 *  @param: address to the joycon msg
 *
 * This function is heavy with goto statements, very simple but rarely used in a high level language, you will need to
 * understand these
 */
void Serial::joystickCallback(const controller::JoyCon::ConstPtr& joy){

    // If we are not using the IR sensors get the joystick data for throttle
    if(!using_ir_sensors){
      m_package[LEFT_DRIVE]  = (joy->LS_UD / (joycon::AXIS_RANGE / 10)) + 11;
      m_package[RIGHT_DRIVE] = (joy->RS_UD / (joycon::AXIS_RANGE / 10)) + 11;
    } else {
       // Leave the packet unchanged
       m_package[LEFT_DRIVE] =  m_package[LEFT_DRIVE];
       m_package[RIGHT_DRIVE] = m_package[RIGHT_DRIVE];
    }

    // Set the other packets to neutral values
    m_package[RAIL]        = 0;
    m_package[DUMP]        = 0;


    if(joy->Y)
      m_package[AUGER_DRIVE] ^= BIT4;    // Toggle speed


    if(joy->SELECT)
      using_ir_sensors = !using_ir_sensors; // Toggle IR Sensors usage


  dump: 
    if((joy->X == 0 && joy->B == 0) ||
       (joy->X != 0 && joy->B != 0)) {
      goto rail;
    }

    if(joy->X){      // Lower Dump
      m_package[DUMP] |= BIT0;
    }
    else if(joy->B){ // Tilt dump
      m_package[DUMP] |= BIT1;
    }
    

  rail:
    if(joy->RS){
      servo_locked = true; 
    }
    else if(joy->LS){
      servo_locked = false;
    }

    if(servo_locked){
      m_package[RAIL] |= BIT4;
    }

    if((joy->DPAD_UD == 0 && joy->DPAD_LR == 0) ||
       (joy->DPAD_UD != 0 && joy->DPAD_LR != 0)){
      goto auger_drive;
    }

    if(joy->DPAD_LR < 0) // RIGHT
      m_package[RAIL] |= BIT0;
    else if(joy->DPAD_LR > 0) // LEFT
      m_package[RAIL] |= BIT1;
    
    if(joy->DPAD_UD > 0) // UP
      m_package[RAIL] |= BIT3;
    else if(joy->DPAD_UD < 0) // DOWN
      m_package[RAIL] |= BIT2;      


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
    // Call the function to send a packet and let it know it is the first try with this packet
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

#include "joystick.hpp"

joycon::JoyCon::JoyCon(const char* path){

    m_js_fd = open(path, O_RDWR);

    m_joy_pub_ = nh_.advertise<controller::JoyCon>("joycon", 1);
    }

void joycon::JoyCon::tryReconnect(const char* path){
     m_js_fd = open(path, O_RDWR);
}

void joycon::JoyCon::checkConnection()
{
    m_bytes = read(m_js_fd, &m_event, sizeof(js_event));

    if(m_bytes == sizeof(js_event)){
        m_connected = true;
        m_is_update = true;
        return;
    }
    if(m_bytes == -1){
        m_connected = false;
    }    
}

void joycon::JoyCon::publishJoyConMessage(){
    switch(m_event.type){
        case JS_EVENT_BUTTON:
            m_buttons[m_event.number] = m_event.value;
            break;
        case JS_EVENT_AXIS:
            m_axes[m_event.number] = m_event.value;
            break;
    }

    controller::JoyCon msg;

    msg.LS_LR   = m_axes[LS_LR];
    msg.LS_UD   = -m_axes[LS_UD];
    msg.LT      = m_axes[LT];
    msg.RS_LR   = m_axes[RS_LR];
    msg.RS_UD   = -m_axes[RS_UD];
    msg.RT      = m_axes[RT];
    msg.DPAD_LR = m_axes[D_LR];
    msg.DPAD_UD = m_axes[D_UD];

    msg.A       = m_buttons[A];
    msg.B       = m_buttons[B];
    msg.X       = m_buttons[X];
    msg.Y       = m_buttons[Y];
    msg.LB      = m_buttons[LB];
    msg.RB      = m_buttons[RB];
    msg.SELECT  = m_buttons[SELECT];
    msg.START   = m_buttons[START];
    msg.XBOX    = m_buttons[XBOX];
    msg.LS      = m_buttons[LS];
    msg.RS      = m_buttons[RS];

    m_joy_pub_.publish(msg);
    m_is_update = false;
}

void joycon::JoyCon::publishDisconnect(){
    controller::JoyCon msg;

    msg.LS_LR   = 0;
    msg.LS_UD   = 0;
    msg.LT      = 0;
    msg.RS_LR   = 0;
    msg.RS_UD   = 0;
    msg.RT      = 0;
    msg.DPAD_LR = 0;
    msg.DPAD_UD = 0;

    msg.A       = 0;
    msg.B       = 0;
    msg.X       = 0;
    msg.Y       = 0;
    msg.LB      = 0;
    msg.RB      = 0;
    msg.SELECT  = 0;
    msg.START   = 0;
    msg.XBOX    = 0;
    msg.LS      = 0;
    msg.RS      = 0;

    m_joy_pub_.publish(msg);
    m_is_update = false;
}

int main(int argc, char *argv[]){

  const char* path;


  if(argc > 1){
    path = argv[1];
  }
  else {
    path = "/dev/input/js0";
  }

  ros::init(argc, argv, "joycon");
  joycon::JoyCon joycon = {path};


  while(joycon.nh_.ok()){
      joycon.checkConnection();
      if(joycon.isConnected()){
        if(joycon.isUpdate()){
            joycon.publishJoyConMessage();
            continue;
        }
        continue;
      }
      joycon.publishDisconnect();
      joycon.tryReconnect(path);
  }


}
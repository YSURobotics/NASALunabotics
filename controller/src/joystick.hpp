#include <iostream>
#include <fcntl.h> 
#include <unistd.h>
#include <linux/joystick.h>
#include <ros/ros.h>
#include <controller/JoyCon.h>

namespace joycon
{
    int AXIS_RANGE = 32767;
    enum AXIS {
        LS_LR = 0,
        LS_UD = 1,
        LT = 2,
        RS_LR = 3,
        RS_UD = 4,
        RT = 5,
        D_LR = 6,
        D_UD = 7
    };

    enum BUTTONS {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        LB = 4,
        RB = 5,
        SELECT = 6,
        START = 7,
        XBOX = 8,
        LS = 9,
        RS = 10
    };

    class JoyCon {
        public:
            JoyCon(const char* path);
            bool isConnected() {return m_connected;}
            bool isUpdate() {return m_is_update;}
            void checkConnection();
            void publishJoyConMessage();
            void publishDisconnect();
            void tryReconnect(const char* path);
            ros::NodeHandle nh_;

        private: 
    

            ros::Publisher m_joy_pub_;


            bool m_connected;
            bool m_is_update;
            int16_t m_axes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            uint8_t m_buttons[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


            js_event m_event;

            int m_js_fd;

            size_t m_bytes;
    };

}
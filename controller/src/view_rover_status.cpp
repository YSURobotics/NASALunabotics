#include "serial.hpp"
#include "colors.hpp"


Rover::Rover() {
    m_rover_sub_ = nh_.subscribe<controller::Rover>("rover", 1, &Rover::messageCallback, this);
    
    now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
}


void Rover::messageCallback(const controller::Rover::ConstPtr& msg) {

    std::system("clear");
    now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    timestamp = ctime(&now);
    timestamp = timestamp.substr(0, timestamp.length() - 1);

    std::cout << timestamp << " Rover Status: \n\n";

        boost_string      = ((msg->auger & BIT4) ? std::string(BOLD(UNDL(FGRN("High")))) + "  " + (FGRY("Low\n")) : std::string(FGRY("High  ")) + BOLD(UNDL(FYEL("Low\n"))));

        rail_right_string = ((msg->rail & BIT0) ? std::string(BOLD(UNDL(FGRN("Engaged")))) + "  " + FGRY("Disengaged\n"): std::string(FGRY("Engaged  ")) + BOLD(UNDL(FRED("Disengaged\n"))));
        rail_left_string  = ((msg->rail & BIT1) ? std::string(BOLD(UNDL(FGRN("Engaged")))) + "  " + FGRY("Disengaged\n"): std::string(FGRY("Engaged  ")) + BOLD(UNDL(FRED("Disengaged\n"))));
        rail_up_string    = ((msg->rail & BIT2) ? std::string(BOLD(UNDL(FGRN("Engaged")))) + "  " + FGRY("Disengaged\n"): std::string(FGRY("Engaged  ")) + BOLD(UNDL(FRED("Disengaged\n"))));
        rail_down_string  = ((msg->rail & BIT3) ? std::string(BOLD(UNDL(FGRN("Engaged")))) + "  " + FGRY("Disengaged\n"): std::string(FGRY("Engaged  ")) + BOLD(UNDL(FRED("Disengaged\n"))));

        auger_right_string = ((msg->auger & BIT0) ? std::string(BOLD(UNDL(FGRN("Drilling")))) + "  " + FGRY("Undrilling  Stopped\n") : (msg->auger & BIT1) ? std::string(FGRY("Drilling")) + "  " + BOLD(UNDL(FYEL("Undrilling"))) + "  "  + FGRY("Stopped\n") : std::string(FGRY("Drilling  ")) + FGRY("Undrilling  ") + FRED("Stopped\n"));
        auger_left_string = ((msg->auger & BIT2) ? std::string(BOLD(UNDL(FGRN("Drilling")))) + "  " + FGRY("Undrilling  Stopped\n") : (msg->auger & BIT3) ? std::string(FGRY("Drilling")) + "  " + BOLD(UNDL(FYEL("Undrilling"))) + "  " + FGRY("Stopped\n") : std::string(FGRY("Drilling  ")) + FGRY("Undrilling  ") + FRED("Stopped\n"));

        dump_string = ((msg->dump & BIT1) ? std::string(BOLD(UNDL(FGRN("Dumping")))) + "  " + FGRY("Lowering  Stopped\n"): (msg->dump & BIT0) ? std::string(FGRY("Dumping  ")) + BOLD(UNDL(FYEL("Lowering"))) + "  " + FGRY("Stopped\n") : std::string(FGRY("Dumping  ")) + FGRY("Lowering  ") + FRED("Stopped\n"));

        std::cout << "\tBoost: " << boost_string;

        std::cout << "\n";

        std::cout << "\tDrivetrain Status: \n";
            std::cout << "\t\tLeft Velocity:  " << (msg->left_vel - 11) << "\n" << 
                         "\t\tRight Velocity: " << (msg->right_vel - 11) << "\n";

        std::cout << "\n";

        std::cout << "\tRail Status: \n";
            std::cout << "\t\tRight Drive:   "   << rail_right_string;
            std::cout << "\t\tLeft  Drive:   "    << rail_left_string; 
            std::cout << "\t\tRotating Up:   "   << rail_up_string; 
            std::cout << "\t\tRotating Down: " << rail_down_string; 

        std::cout << "\n";

        std::cout << "\tAuger Status: \n";
            std::cout << "\t\tRight Auger: " << auger_right_string;
            std::cout << "\t\tLeft  Auger: "  << auger_left_string;
            
        std::cout << "\n";

            std::cout << "\tDump Status: \n";
            std::cout << "\t\tDump Bin: " << dump_string;


        std::cout << "\n";
}





int main(int argc, char** argv)
{

  Rover rov = {};
  ros::init(argc, argv, "rover_stream");

  ros::spin();
}

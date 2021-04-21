#include "serial.hpp"
#include "colors.hpp"


Rover::Rover() {
    m_rover_sub_ = nh_.subscribe<controller::Rover>("rover", 1, &Rover::messageCallback, this);
}


void Rover::messageCallback(const controller::Rover::ConstPtr& msg) {

    std::system("clear");
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string timestamp = ctime(&now);
    timestamp = timestamp.substr(0, timestamp.length() - 1);

    std::cout << timestamp << " Rover Status: \n";

        std::cout << "\tBoost: " << ((msg->auger & BIT4) ? (BOLD(UNDL(FGRN("High"))) << FGRY("Low\n"): (FGRY("High") << BOLD(UNDL(FYEL("Low\n"))))));

        std::cout << "\tDrivetrain Status: \n";
            std::cout << "\t\tLeft Velocity: " << msg->left_vel << "\n" << 
                         "\t\tRight Velocity: " << msg->right_vel << "\n";

        std::cout << "\n";

        std::cout << "\tRail Status: \n";
            std::cout << "\t\tRight Drive: "   << ((msg->rail & BIT0) ? (BOLD(UNDL(FGRN("Engaged"))) << FGRY("Disengaged\n"): (FGRY("Engaged") << BOLD(UNDL(FRED("Disengaged\n")))))); 
            std::cout << "\t\tLeft Drive: "    << ((msg->rail & BIT1) ? (BOLD(UNDL(FGRN("Engaged"))) << FGRY("Disengaged\n"): (FGRY("Engaged") << BOLD(UNDL(FRED("Disengaged\n")))))); 
            std::cout << "\t\tRotating Up: "   << ((msg->rail & BIT2) ? (BOLD(UNDL(FGRN("Engaged"))) << FGRY("Disengaged\n"): (FGRY("Engaged") << BOLD(UNDL(FRED("Disengaged\n")))))); 
            std::cout << "\t\tRotating Down: " << ((msg->rail & BIT3) ? (BOLD(UNDL(FGRN("Engaged"))) << FGRY("Disengaged\n"): (FGRY("Engaged") << BOLD(UNDL(FRED("Disengaged\n")))))); 

        std::cout << "\n";

        std::cout << "\tAuger Status: \n";
            std::cout << "\t\tRight Auger: " << ((msg->auger & BIT0) ? (BOLD(UNDL(FGRN("Drilling"))) << FGRY("Undrilling Stopped\n")): (msg->auger & BIT1) ? (FGRY("Drilling") << BOLD(UNDL(FYEL("Undrilling"))) << FGRY("Stopped\n")) : (FGRY("Drilling") << FGRY("Undrilling") << FRED("Stopped\n")));
            std::cout << "\t\tLeft Auger: "  << ((msg->auger & BIT2) ? (BOLD(UNDL(FGRN("Drilling"))) << FGRY("Undrilling Stopped\n")): (msg->auger & BIT3) ? (FGRY("Drilling") << BOLD(UNDL(FYEL("Undrilling"))) << FGRY("Stopped\n")) : (FGRY("Drilling") << FGRY("Undrilling") << FRED("Stopped\n")));
           
        std::cout << "\n";

        std::cout << "\tDump Status: \n";
            std::cout << "\t\tDump Bin: " << ((msg->dump & BIT1) ? (BOLD(UNDL(FGRN("Dumping"))) << FGRY("Lowering Stopped\n")): (msg->dump & BIT0) ? (FGRY("Dumping") << BOLD(UNDL(FYEL("Lowering"))) << FGRY("Stopped\n")) : (FGRY("Dumping") << FGRY("Lowering") << FRED("Stopped\n")));


        std::cout << "\n";
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover_stream");

  ros::spin();
}

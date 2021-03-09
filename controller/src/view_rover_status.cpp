#include "serial.hpp"
#include "colors.hpp"


Rover::Rover() {
    m_rover_sub_ = nh_.subscribe<controller::Rover>("rover", 1, &Rover::messageCallback, this);
}


void Rover::messageCallback(const controller::Rover::ConstPtr& msg) {

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string timestamp = ctime(&now);
    timestamp = timestamp.substr(0, timestamp.length() - 1);

    std::cout << timestamp << " Rover Status: \n";

        std::cout << "\tDrivetrain Status: \n";
            std::cout << "\t\tLeft Velocity: " << msg->left_vel << "\n" << 
                         "\t\tRight Velocity: " << msg->right_vel << "\n";

        std::cout << "\n";

        std::cout << "\tRail Status: \n";
            std::cout << "\t\tRight Drive: "   << ((msg->rail & BIT0) ? FGRN("Engaged\n") : FRED("Disengaged\n")); 
            std::cout << "\t\tLeft Drive: "    << ((msg->rail & BIT1) ? FGRN("Engaged\n") : FRED("Disengaged\n"));
            std::cout << "\t\tRotating Up: "   << ((msg->rail & BIT2) ? FGRN("Engaged\n") : FRED("Disengaged\n"));
            std::cout << "\t\tRotating Down: " << ((msg->rail & BIT3) ? FGRN("Engaged\n") : FRED("Disengaged\n"));

        std::cout << "\n";

        std::cout << "\tAuger Status: \n";
            std::cout << "\t\tRight Auger: " << ((msg->auger & BIT0) ? FGRN("Drilling\n") : (msg->auger & BIT1) ? FRED("Undrilling\n") : "Stopped\n");
            std::cout << "\t\tLeft Auger: "  << ((msg->auger & BIT2) ? FGRN("Drilling\n") : (msg->auger & BIT3) ? FRED("Undrilling\n") : "Stopped\n");
            std::cout << "\t\tSpeed: "       << ((msg->auger & BIT4) ? FGRN("High\n") : FYEL("Low\n"));

        std::cout << "\n";

        std::cout << "\tDump Status: \n";
            std::cout << "\t\tDump Bin: " << ((msg->dump & BIT1) ? FGRN("Dumping\n") : (msg->dump & BIT0) ? FRED("Lowering\n") : "Stopped\n");


        std::cout << "\n";
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover_stream");

  ros::spin();
}

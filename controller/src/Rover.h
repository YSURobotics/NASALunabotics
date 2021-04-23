
class Rover{

public:
    Rover();
    
private:

    void messageCallback(const controller::Rover::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber m_rover_sub_;

    uint8_t m_package[8] = {0,0,0,0,0,0,0,0};

    std::string timestamp = " ";

    std::string boost_string = " ";

    std::string drivetrain_string = " "; 

    std::string rail_right_string = " ";
    std::string rail_left_string = " ";
    std::string rail_up_string = " ";
    std::string rail_down_string = " ";

    std::string auger_right_string = " ";
    std::string auger_left_string = " ";

    std::string dump_string = " ";
    
    std::time_t now;
};


#include <ros/ros.h>
#include <controller/Rover.h>
#include <sensor_msgs/Joy.h>


class Controller
{
public:
  Controller();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


Controller::Controller()
{
  vel_pub_ = nh_.advertise<controller::Rover>("controller/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::joyCallback, this);

}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  controller::Rover data;

  double left_stick = joy->axes[1];
  double right_stick = joy->axes[4];

  std::cout << left_stick << " , " << right_stick << std::endl;

  data.left_vel = (left_stick * 10) + 10;
  data.right_vel = (right_stick * 10) + 10;

  vel_pub_.publish(data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  Controller teleop_turtle;

  ros::spin();
}
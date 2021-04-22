# Lunabotics

**_ REQUIRES UBUNTU 18.04 _**

Other Debian distributions may work but are untested.

Step 1) Install ROS

In a terminal run the following commands:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    sudo apt update

    sudo apt install ros-melodic-desktop-full

(Optional) If you would like to source ROS with each new terminal instance run:

    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

OR

If you would like to do it each time yourself; run the following command in each new terminal instance:

    source /opt/ros/melodic/setup.bash

REQUIRED

    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

    sudo apt install python-rosdep

    sudo rosdep init
    rosdep update

STEP 2)

Download other dependencies

VIDEO STREAM ROS PACKAGE

    sudo apt-get install ros-melodic-video-stream-opencv

STEP 3)

Create a Catkin Workspace

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

Step 4)

Clone this repo into catkin_ws/src

    cd ~/catkin_ws/src/
    git clone https://github.com/YSURobotics/NASALunabotics.git

Step 5)

Build

    cd ~/catkin_ws/
    catkin_make

To source this library in ROS you have to run

      . ~/catkin_ws/devel/setup.bash

in each terminal instance you'd like to use this library

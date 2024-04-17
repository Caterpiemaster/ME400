# ME400
repo for capstone
made to control robot for capstone design class with raspberry pi using arduino for motor control and encoder

## Requirements
This repository requires Ubuntu 22.04 and ROS2 Humble.

## Setting up this repository 
This is a repository to simulate and control capstone robot. currently, we only tested with one motor.
~~~~bash
cd ~
mkdir -p capstone_ws/src
cd capstone_ws/src/
git clone https://github.com/Caterpiemaster/ME400.git
~~~~

Install additional dpendency.
~~~~bash
sudo apt install -y \
ros-humble-hardware-interface \
ros-humble-controller-manager \
ros-humble-control-msgs \
ros-humble-xacro \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-joint-state-publisher \
ros-humble-joint-state-publisher-gui \
ros-humble-gazebo-ros2-control
~~~~
If you have some error when you use launch.py file, please let me know.

Then build package.

~~~~bash
cd ~/capstone_ws
colcon build --symlink-install
source ./install.sh
~~~~

now launch robot description and spawn controller
~~~~bash
sudo chmod 666 /dev/ttyACM0
ros2 launch diffdrive_arduino capstone.launch.py use_fake_hardware:=false
~~~~
then visualize with rviz.
~~~~bash
rviz2
~~~~
add robot_description topic in rviz. then change fixed frame in global options to 'world'

if you want to just test urdf and spawning controller, (if you want to check all arguments, use --show-args)
if you use fake_hardware, although you visualize robot description in rviz, it will not rotate.
~~~~bash
ros2 launch diffdrive_arduino capstone.launch.py use_fake_hardware:=true
~~~~

if you want to just test robot description and spawning controller, (if you want to check all arguments, use --show-args)
if you use fake_hardware, although you visualize robot description in rviz, it will not rotate.
~~~~bash
ros2 launch diffdrive_arduino capstone.launch.py use_fake_hardware:=true
~~~~

if you want to run robot in gazebo, use following command
~~~~bash
ros2 launch diffdrive_arduino gazebo_capstone.launch.py
~~~~

to control velocity of motor, open another terminal
~~~~bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [{label: 'single', size: 1, stride: 1}], data_offset: 0}, data: [0.0]}"
~~~~
change target velocity in data: [0,0]. maximum is 10
# Remote Control

Let's start remote control with gamepad

## How to connect gamepad with ros2

- connect your gamepad with pc (bluetooth, usb, ... )
- download some library to check joystick connection
    ~~~~bash
    sudo apt install joystick jstest-gtk evtest
    evtest
    jstest-gtk
    ~~~~
- in ros2, publish joy message
    ~~~~bash
    ros2 run joy joy_enumerate_devices # check gamepad connection
    ros2 run joy joy_node # publish joy message
    ~~~~
    open another terminal
    ~~~~bash
    ros2 topic echo /joy
    ~~~~
- for diff_drive_controller, I made joystick.launch.py to publish twist message
    ~~~~bash
    ros2 launch diffdrive_arduino joystick.launch.py 
    # run joy joy_node
    # run teleop_twist_joy teleop_node
    ~~~~
    check which topic is being published to controller
    ~~~~bash
    ros2 topic echo /diff_drive_base_controller/cmd_vel_unstamped
    ~~~~    
## Test Teleoperation
- to test teleoperation, I made gazebo_capstone_test.launch.py
    ~~~~bash
    ros2 launch diffdrive_arduino gazebo_capstone_test.launch.py
    ~~~~    
    if you running joystick.launch.py, you can teleoperate robot.
    
    Keep push R button to publish twist message.
    if you want to enable turbo, push L button. 

    Tilt left joystick to move robot.

    up, down -> control forward direction linear velocity

    left, right -> control angular velocity

    The buttons could be different if you are using different gamepad with me. (I'm using Nintendo switch Pro Controller)
    Please check config/joystick.yaml if you want to change button.
Updated to reflect a six wheel robot with firmware 0.5.1

install
sudo apt-get install ros-melodic-diff-drive-controller
sudo apt-get install ros-melodic-controller-manager
sudo apt-get install ros-melodic-joint-state-controller

to run 
roslaunch odrive_cpp_ros control.launch

# odrive_cpp_ros
A ros_control based c++ driver to control 4 hoverboard/hub motors using 2 odrive boards. It is intended for skid steer style robots such as the Clearpath Husky. The hardware interface was created following the ros_control tutorial [here](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface). I updated the original repo to work with my set up. Updated the odometry position calculations and added a watchdog for safety (which doesn't work perfectly yet). Also added a Battery Voltage Publisher. This repo is still a work in progress.


### How To Run
You will need to adjust two parameters in hardware_interface.cpp file:
  - **Encoder CPR**: Check your encoder's CPR and adjust the ENCODER_CPR value accordingly to get accurate odometry. My motor had an encoder resolution of 90 counts and a 5:1 gear ratio, so for one revolution of the wheel the motor spins 5 times, therefore 1 wheel revolution = 90 * 5 = 450 counts.
  - **Serial No.**: Check your two odrive board serial numbers and update the values accordingly.
  
You will also need to run the file `generate_odrive_header.sh` in tools, which should automatically update the file `odrive_endpoints.h` to work with your setup/firmware, and modify the control.yaml file to configure the diff drive controller for your robot (eg. change controller name and parameters to your liking).

Finally, odometry in a skid steer robot will never be super accurate when using a differential drive kinematic model, for that reason, you can play around with the wheel_separation_multiplier increasing it until you get somewhat accurate performance.

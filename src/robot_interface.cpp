
//Load hardware interface stuff
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//Load controller manager
#include "controller_manager/controller_manager.h"

//Load odrive stuff
#include "odrive_cpp_ros/odrive_cpp_ros.h"
#include "ros/callback_queue.h"


#define FRONT_ODRIVE_SERIAL "35563259515441"
#define MIDDLE_ODRIVE_SERIAL  "35567554548273"
#define BACK_ODRIVE_SERIAL  "35627684221489"
#define ENCODER_CPR 90 //Count per revolution
#define PI 3.14159265358979
#define TWO_PI (2 * PI)
#define RAD_PER_CPR (TWO_PI / ENCODER_CPR)

class Navvy : public hardware_interface::RobotHW {
    public:
    Navvy() {
        hardware_interface::JointStateHandle state_handle_front_left("front_left_wheel", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle state_handle_front_right("front_right_wheel", &pos[1], &vel[1], &eff[1]);
        hardware_interface::JointStateHandle state_handle_middle_left("middle_left_wheel", &pos[2], &vel[2], &eff[2]);
        hardware_interface::JointStateHandle state_handle_middle_right("middle_right_wheel", &pos[3], &vel[3], &eff[3]);
        hardware_interface::JointStateHandle state_handle_rear_left("rear_left_wheel", &pos[4], &vel[4], &eff[4]);
        hardware_interface::JointStateHandle state_handle_rear_right("rear_right_wheel", &pos[5], &vel[5], &eff[5]);
        
        jnt_state_interface.registerHandle(state_handle_front_left);
        jnt_state_interface.registerHandle(state_handle_front_right);
        jnt_state_interface.registerHandle(state_handle_middle_left);
        jnt_state_interface.registerHandle(state_handle_middle_right);
        jnt_state_interface.registerHandle(state_handle_rear_left);
        jnt_state_interface.registerHandle(state_handle_rear_right);

        registerInterface(&jnt_state_interface);

        hardware_interface::JointHandle vel_handle_front_left(jnt_state_interface.getHandle("front_left_wheel"), &cmd[0]);
        hardware_interface::JointHandle vel_handle_front_right(jnt_state_interface.getHandle("front_right_wheel"), &cmd[1]);
        hardware_interface::JointHandle vel_handle_middle_left(jnt_state_interface.getHandle("middle_left_wheel"), &cmd[2]);
        hardware_interface::JointHandle vel_handle_middle_right(jnt_state_interface.getHandle("middle_right_wheel"), &cmd[3]);
        hardware_interface::JointHandle vel_handle_rear_left(jnt_state_interface.getHandle("rear_left_wheel"), &cmd[4]);
        hardware_interface::JointHandle vel_handle_rear_right(jnt_state_interface.getHandle("rear_right_wheel"), &cmd[5]);
        
        for (int i = 0; i < 6; ++i) {
            cmd[i] = 0.0;
        }

        jnt_vel_interface.registerHandle(vel_handle_front_left);
        jnt_vel_interface.registerHandle(vel_handle_front_right);
        jnt_vel_interface.registerHandle(vel_handle_middle_left);
        jnt_vel_interface.registerHandle(vel_handle_middle_right);
        jnt_vel_interface.registerHandle(vel_handle_rear_left);
        jnt_vel_interface.registerHandle(vel_handle_rear_right);

        registerInterface(&jnt_vel_interface); //up to here it follows the standard stuff from ros tutorial

        //Create an array with 3 strings, which are the odrive serial numbers
        std::string ser_nums[3] = {FRONT_ODRIVE_SERIAL, MIDDLE_ODRIVE_SERIAL, BACK_ODRIVE_SERIAL};
        //Create and array of strings, assign odrive to each motor FL, FR, ML, MR, BL, BR
        std::string set_motor_map[6] = {FRONT_ODRIVE_SERIAL, FRONT_ODRIVE_SERIAL, MIDDLE_ODRIVE_SERIAL, MIDDLE_ODRIVE_SERIAL, BACK_ODRIVE_SERIAL, BACK_ODRIVE_SERIAL};
        //Create an array defining the motor index to be used by odrive - either 0 or 1 - Remember when wiring up
        uint8_t motor_indexes[6] = {1, 0, 0, 1, 1, 0};
        //Create new motor driver for 3 odrives and initialise it. If it starts up, set motors enabled to true.
        motor_driver = new odrive::ODriveDriver(ser_nums, 3, set_motor_map, motor_indexes, 6);
        int result = motor_driver->init();
        if (result != 0) {
            std::cout << "Could not connect to odrives!"<< std::endl;
            motors_enabled = false;
        } else {
            motor_driver->sendWatchdog();
            motor_driver->setErrors();
            motor_driver->setStates(); 
            std::cout << "Connected to odrives!"<< std::endl;
            motors_enabled = true; //If connected to odrive, set motors_enable to true
            
        }
    }

    //Get encoder readings
    void updateJointsFromHardware() {
        //std::cout << "Updating joints" << std::endl;
        float speed;
	float motor_pos;
        float pos_cpr;
        float pos_delta;
        double angle_delta;
        //For each joint
        for (int i = 0; i < 6; ++i) {
            //get speed of motor
            motor_driver->getMotorSpeed(i, speed);
            //Get speed reading for each wheel - multiply speed in turn/s to get it in rad/s
            vel[i] = ((speed * direction_multipliers[i]) ) * 2 * 3.141592;
            //std::cout << "speed: " << vel[i] << std::endl; so this only goes positive and negative (no switching)
            
            motor_driver->getMotorPosition(i, motor_pos); //in turns
            
            if (first_pos_yet){
            	first_motor_pos[i] = motor_pos;
            	std::cout << "first motor pos : " << first_motor_pos[i] << std::endl;
            }
            
            pos[i] = (((motor_pos-first_motor_pos[i]) * direction_multipliers[i])) * 2 * 3.141592;
            //std::cout << "position: " << pos[i] << std::endl; // so this only goes positive and negative (no switching)
            
            //Get position from encoders
            //motor_driver->getPosCPR(i, pos_cpr); //In counts per revolution
            	//If previously had encoder readings
            //if (last_cpr_populated) {
                //change in pos = current reading vs last reading
                //pos_delta = pos_cpr - last_pos_cpr[i];
                //std::cout << "pos delta : " << pos_delta << std::endl;
                
                //if (pos_delta < - ENCODER_CPR / 2.0) { //If position delta smaller than -45 cpr (ie. -180) transform it to positive
                    //overflow will go from eg 80 to 10. Delta = -70
                    // want to add on ENCODER_CPR to change to delta = 20
                    //pos_delta += ENCODER_CPR;
                //} else if (pos_delta > ENCODER_CPR / 2.0) { //If position delta bigger than 45 cpr (ie. 180) transform it to negative
                    //underflow will go from eg 10 to 80, Delta = 70
                 //   pos_delta -= ENCODER_CPR;
                //}
                //pos_delta *= direction_multipliers[i]; //Multiply to get direction of wheels correct 
                //angle_delta = pos_delta * RAD_PER_CPR; // Transform to angle in radians

                //Position in radians
                //pos[i] = pos[i] + angle_delta; //Initially pos will be 0 (Setting initial offset). Then pos added incrementally.

                //std::cout << "Motor " << i << " pos " << pos[i] << " delta: " << angle_delta << std::endl;

            //}         
            //last_pos_cpr[i] = pos_cpr; //exit loop and save reading as latest

            // int motor_pos;
            // motor_driver->readCurrentMotorPosition(i, motor_pos);
            // std::cout << "sad: " << i << " : " << motor_pos << std::endl;
            // motor_driver->getMotorPosition(i, motor_pos);
            
        }
        //last_cpr_populated = true; //After first reading set variable to true
        first_pos_yet = false; 
    }   

    //Write commands to motors
    void writeCommandsToHardware() {
 
        float target_speeds[6];//Create target speeds array of floats
        //For each wheel
        for (int i = 0; i < 6; ++i) {
            //If command too big or too small - ignore. Why 200?
            if (cmd[i] < -16 || cmd[i] > 16) { //2.5 turns/s (or 15.7 rad/s) equate to 3mph for 17cm diameter wheel.
                std::cout << "Motor speed request: " << cmd[i] << " ignored." << std::endl;
                target_speeds[i] = 0.0;
            } else {
                target_speeds[i] = cmd[i] / TWO_PI * direction_multipliers[i]; //Convert from rad/s to turns/s (Divide by 2pi to get rev), correct direction, save to target speeds
            }
            //std::cout << target_speeds[i] << " ";
        }
         //std::cout << std::endl;
        
        //if motors connected
        if (motors_enabled) {
            //std::cout << "motor enabled, receiving cmds" << std::cout;
            ///motor_driver->sendWatchdog();
            ///motor_driver->setMotorSpeeds(target_speeds);
        }
    }

    //This seems to follow the ROS tutorial but check the lower parts
    private:
        odrive::ODriveDriver *motor_driver;
        hardware_interface::JointStateInterface jnt_state_interface; //Define here and used at the top
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        //Define arrays and set to 0
        double cmd[6] = {0, 0, 0, 0, 0, 0}; //Initially set to 0. Controller Manager gets this from diff drive controller
        double pos[6] = {0, 0, 0, 0, 0, 0}; //Controller manager will read these and feed to controller
        double vel[6] = {0, 0, 0, 0, 0, 0};
        double eff[6] = {0, 0, 0, 0, 0, 0};

        //float last_pos_cpr[6]; //Create an array of 6 floats for last position CPR of each wheel use above
        float first_motor_pos[6];
        //bool last_cpr_populated = false; //Create last_cpr_populated variable and set to false initally
	    bool first_pos_yet = true;
	
        int direction_multipliers[6] = {1, -1, -1, 1, 1, -1}; //Define direction to turn - used at top
        bool motors_enabled; //Create motors enabled variable which will be true when connected

};


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "robot_odrive_interface");
    Navvy robot;
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); //Is this rate enough?
    while(ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        robot.updateJointsFromHardware();
        cm.update(time, period); //Not 100% sure how the timing stuff works and what the best way to do it is.
        robot.writeCommandsToHardware();
        rate.sleep();
    }
    return 0;
}


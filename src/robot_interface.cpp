
//Load hardware interface stuff
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//Load controller manager
#include "controller_manager/controller_manager.h"

//Load odrive stuff
#include "odrive_cpp_ros/odrive_cpp_ros.h"
#include "ros/callback_queue.h"

#include <signal.h>


#define ENCODER_CPR 90 //Count per revolution
#define PI 3.14159265358979
#define TWO_PI (2 * PI)
#define RAD_PER_CPR (TWO_PI / ENCODER_CPR)

class Booba : public hardware_interface::RobotHW {
    public:
    Booba(std::string FRONT_ODRIVE_SERIAL, std::string LEFT_ODRIVE_SERIAL, std::string RIGHT_ODRIVE_SERIAL, int number_of_wheels) {
        robot_number_of_wheels = number_of_wheels;

        if (number_of_wheels == 6) {

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
            std::string ser_nums[3] = {FRONT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
            //Create and array of strings, assign odrive to each motor FL, FR, ML, MR, BL, BR
            std::string set_motor_map[6] = {FRONT_ODRIVE_SERIAL, FRONT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
            //Create an array defining the motor index to be used by odrive - either 0 or 1 - Remember when wiring up
            uint8_t motor_indexes[6] = {1, 0, 0, 1, 1, 0};
            //Create new motor driver for 3 odrives and initialise it. If it starts up, set motors enabled to true.
            motor_driver = new odrive::ODriveDriver(ser_nums, 3, set_motor_map, motor_indexes, 6);

        //#endif
        }
        else if (number_of_wheels == 4){

            hardware_interface::JointStateHandle state_handle_front_left("front_left_wheel", &pos[0], &vel[0], &eff[0]);
            hardware_interface::JointStateHandle state_handle_front_right("front_right_wheel", &pos[1], &vel[1], &eff[1]);
            hardware_interface::JointStateHandle state_handle_middle_left("middle_left_wheel", &pos[2], &vel[2], &eff[2]);
            hardware_interface::JointStateHandle state_handle_middle_right("middle_right_wheel", &pos[3], &vel[3], &eff[3]);
            
            jnt_state_interface.registerHandle(state_handle_front_left);
            jnt_state_interface.registerHandle(state_handle_front_right);
            jnt_state_interface.registerHandle(state_handle_middle_left);
            jnt_state_interface.registerHandle(state_handle_middle_right);

            registerInterface(&jnt_state_interface);

            hardware_interface::JointHandle vel_handle_front_left(jnt_state_interface.getHandle("front_left_wheel"), &cmd[0]);
            hardware_interface::JointHandle vel_handle_front_right(jnt_state_interface.getHandle("front_right_wheel"), &cmd[1]);
            hardware_interface::JointHandle vel_handle_middle_left(jnt_state_interface.getHandle("middle_left_wheel"), &cmd[2]);
            hardware_interface::JointHandle vel_handle_middle_right(jnt_state_interface.getHandle("middle_right_wheel"), &cmd[3]);
            
            for (int i = 0; i < 4; ++i) {
                cmd[i] = 0.0;
            }

            jnt_vel_interface.registerHandle(vel_handle_front_left);
            jnt_vel_interface.registerHandle(vel_handle_front_right);
            jnt_vel_interface.registerHandle(vel_handle_middle_left);
            jnt_vel_interface.registerHandle(vel_handle_middle_right);

            registerInterface(&jnt_vel_interface); //up to here it follows the standard stuff from ros tutorial

            //Create an array with 3 strings, which are the odrive serial numbers
            std::string ser_nums[3] = {FRONT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
            //Create and array of strings, assign odrive to each motor FL, FR, ML, MR, BL, BR
            std::string set_motor_map[4] = {FRONT_ODRIVE_SERIAL, FRONT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
            //Create an array defining the motor index to be used by odrive - either 0 or 1 - Remember when wiring up
            uint8_t motor_indexes[4] = {1, 0, 0, 1};
            //Create new motor driver for 3 odrives and initialise it. If it starts up, set motors enabled to true.
            motor_driver = new odrive::ODriveDriver(ser_nums, 3, set_motor_map, motor_indexes, 4);
            
        //#endif
        }

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
    void updateJointsFromHardware(int number_of_wheels) {
        //std::cout << "Updating joints" << std::endl;
        float speed;
	float motor_pos;
        float pos_cpr;
        float pos_delta;
        double angle_delta;
        //For each joint
        for (int i = 0; i < number_of_wheels; ++i) {
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
                
        }

        //last_cpr_populated = true; //After first reading set variable to true
        first_pos_yet = false; 
    }   

    //Write commands to motors
    void writeCommandsToHardware(int number_of_wheels) {
 
        float target_speeds[number_of_wheels];//Create target speeds array of floats
        //For each wheel

        for (int i = 0; i < number_of_wheels; ++i) {
            //If command too big or too small - ignore. Why 200?
            if (cmd[i] < -20 || cmd[i] > 20) { //2.5 turns/s (or 15.7 rad/s) equate to 3mph for 17cm diameter wheel.
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
            //std::cout << "motor enabled, receiving cmds" << std::endl;
            motor_driver->sendWatchdog();
            motor_driver->setMotorSpeeds(target_speeds);
        }
    }

    ~Booba(){
        std::cout << "killing odrives!"<< std::endl;
        if (motors_enabled) {
            float target_speeds[robot_number_of_wheels];
            for (int i = 0; i < robot_number_of_wheels; ++i) {
                target_speeds[i] = 0.0;
            }
            motor_driver->setMotorSpeeds(target_speeds);
            motor_driver->sendWatchdog();
            motor_driver->setErrors();
            motor_driver->setIdleStates(); 
            std::cout << "disconnected from odrives!"<< std::endl;
            motors_enabled = false; 
        }
    }

    //This seems to follow the ROS tutorial but check the lower parts
    private:
        odrive::ODriveDriver *motor_driver;
        hardware_interface::JointStateInterface jnt_state_interface; //Define here and used at the top
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        //Define arrays and set to 0
                
/*         #ifdef FOUR_WHEELS
            double cmd[4] = {0, 0, 0, 0}; //Initially set to 0. Controller Manager gets this from diff drive controller
            double pos[4] = {0, 0, 0, 0}; //Controller manager will read these and feed to controller
            double vel[4] = {0, 0, 0, 0};
            double eff[4] = {0, 0, 0, 0};
            float first_motor_pos[4];
            int direction_multipliers[4] = {1, -1, 1, -1}; //Define direction to turn - used at top
        #endif */
        
        double cmd[6] = {0, 0, 0, 0, 0, 0}; //Initially set to 0. Controller Manager gets this from diff drive controller
        double pos[6] = {0, 0, 0, 0, 0, 0}; //Controller manager will read these and feed to controller
        double vel[6] = {0, 0, 0, 0, 0, 0};
        double eff[6] = {0, 0, 0, 0, 0, 0};

        float first_motor_pos[6];
        int direction_multipliers[6] = {1, -1, 1, -1, 1, -1}; //Define direction to turn - used at top
        

        //float last_pos_cpr[6]; //Create an array of 6 floats for last position CPR of each wheel use above
        //bool last_cpr_populated = false; //Create last_cpr_populated variable and set to false initally
	    bool first_pos_yet = true;
	
        bool motors_enabled; //Create motors enabled variable which will be true when connected

        int robot_number_of_wheels = 6;

};


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "robot_odrive_interface");
    ros::NodeHandle nh;
    std::string front_odrive_serial;
    std::string left_odrive_serial;
    std::string right_odrive_serial;
    int number_of_wheels;
    float run_rate = 10.0;
    ROS_INFO("loading parameters");
    nh.getParam("/robot_hardware_interface/front_odrive_serial",front_odrive_serial);
    nh.getParam("/robot_hardware_interface/left_odrive_serial",left_odrive_serial);
    nh.getParam("/robot_hardware_interface/right_odrive_serial",right_odrive_serial);
    nh.getParam("/robot_hardware_interface/number_of_wheels",number_of_wheels);
    nh.getParam("/robot_hardware_interface/run_rate",run_rate);
    

    Booba robot(front_odrive_serial, left_odrive_serial, right_odrive_serial, number_of_wheels);
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    //signal(SIGINT, robot.SigintHandler());

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(run_rate); //Is this rate enough?
    while(ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        //robot.updateJointsFromHardware(number_of_wheels);
        cm.update(time, period); //Not 100% sure how the timing stuff works and what the best way to do it is.
        robot.writeCommandsToHardware(number_of_wheels);
        rate.sleep();
    }
    return 0;
}


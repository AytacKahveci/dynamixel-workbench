/*
@author: Aytaç Kahveci

This script enables to find dynamixel limit positions. Because dynamixel runs in the multi-turn mode it is necessary
to find these limits for safe working conditions.

After the limits found, these values saved to the configuration file.

This script works with dynamixel_limits_interface.cpp because it requires the limited jogging to the motors while 
finding the limits.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamixel_custom_controller/dynamixel_class.h>

#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <fstream>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <signal.h>
#include <stdlib.h>
#include <map>

#define MOTOR1_RES 78.615
#define MOTOR2_RES 76.92

namespace dynamixel_limits
{
    class DynamixelLimits
    {

    public:
        DynamixelLimits(ros::NodeHandle& nh): nh_(nh)
        {
            output_path_ = ros::package::getPath("dynamixel_custom_controller") + "/config/limits.yaml";
            ROS_INFO("Output path:%s", output_path_.c_str());

            pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/dynamixel/JointPosGroup/poseffCommand",1);
            
            dynamixel_states_.reset(new dynamixel_class::DynamixelClass(nh_, 2));

            dynamixel_command_.data.resize(4);
        }

        ~DynamixelLimits()
        { }

        void run()
        {
            int bytes_read = 0;
            bytes_read = read(STDIN_FILENO, &c, 1);
            if(bytes_read != 1)
            {
                return;
            }

            switch(c)
            {
                case 'f': //ID1 Forward Movement
                {
                    dynamixel_command_.data[0] = dynamixel_states_->position[0] + increment * MOTOR1_RES;
                    dynamixel_command_.data[1] = dynamixel_states_->position[1];
                    dynamixel_command_.data[2] = 5;
                    dynamixel_command_.data[3] = 5;
                    pub_.publish(dynamixel_command_);
                    key_ = "/motor1/forward_limit";
                    break;
                }
                case 'g': //ID1 Backward Movement
                {
                    dynamixel_command_.data[0] = dynamixel_states_->position[0] - increment * MOTOR1_RES;
                    dynamixel_command_.data[1] = dynamixel_states_->position[1];
                    dynamixel_command_.data[2] = 5;
                    dynamixel_command_.data[3] = 5;
                    pub_.publish(dynamixel_command_);
                    key_ = "/motor1/backward_limit";
                    break;
                }
                case 'v': //ID2 Forward Movement
                {
                    dynamixel_command_.data[0] = dynamixel_states_->position[0];
                    dynamixel_command_.data[1] = dynamixel_states_->position[1] + increment * MOTOR2_RES;
                    dynamixel_command_.data[2] = 5;
                    dynamixel_command_.data[3] = 5;
                    pub_.publish(dynamixel_command_);
                    key_ = "/motor2/forward_limit";
                    break;
                }
                case 'b': //ID2 Backward Movement
                {
                    dynamixel_command_.data[0] = dynamixel_states_->position[0];
                    dynamixel_command_.data[1] = dynamixel_states_->position[1] - increment * MOTOR2_RES;
                    dynamixel_command_.data[2] = 5;
                    dynamixel_command_.data[3] = 5;
                    pub_.publish(dynamixel_command_);
                    key_ = "/motor2/backward_limit";
                    break;
                }
                case 't': //Increase the increment value
                {
                    increment += 0.5;
                    if(increment > 5)
                    {
                        increment = 5;
                    }
                    std::cout << "Increment: " << increment << "mm" << std::endl;
                    break;
                }
                case 'r': //Decrease the increment value
                {
                    increment -= 0.5;
                    if(increment < 0.5)
                    {
                        increment = 0.5;
                    }
                    std::cout << "Increment: " << increment << "mm" << std::endl;
                    break;
                }
                case 'a':
                {
                    if(key_ == "/motor1/forward_limit")
                    {
                        limits_[key_] = static_cast<int>(dynamixel_states_->position[0]);
                        std::cout << "Forward limit for ID1: " << static_cast<int>(dynamixel_states_->position[0]) << std::endl;
                        break;
                    }
                    else if(key_ == "/motor1/backward_limit")
                    {
                        limits_[key_] = static_cast<int>(dynamixel_states_->position[0]);
                        std::cout << "Backward limit for ID1: " << static_cast<int>(dynamixel_states_->position[0]) << std::endl;
                        break;
                    }
                    else if(key_ == "/motor2/forward_limit")
                    {
                        limits_[key_] = static_cast<int>(dynamixel_states_->position[1]);
                        std::cout << "Forward limit for ID2: " << static_cast<int>(dynamixel_states_->position[1]) << std::endl;
                        break;
                    }
                    else if(key_ == "/motor2/backward_limit")
                    {
                        limits_[key_] = static_cast<int>(dynamixel_states_->position[1]);
                        std::cout << "Backward limit for ID2: " << static_cast<int>(dynamixel_states_->position[1]) << std::endl;
                        break;
                    }
                    else
                    {
                        std::cout << "Key element is empty" << std::endl;
                        break;
                    }
                }
                case 's':
                {
                    auto search = limits_.find("/motor1/forward_limit");
                    if(search == limits_.end())
                    {
                        ROS_WARN("Forward limit for ID1 is empty");
                        break;
                    }
                    search = limits_.find("/motor1/backward_limit");
                    if(search == limits_.end())
                    {
                        ROS_WARN("Backward limit for ID1 is empty");
                        break;
                    }
                    search = limits_.find("/motor2/forward_limit");
                    if(search == limits_.end())
                    {
                        ROS_WARN("Forward limit for ID2 is empty");
                        break;
                    }
                    search = limits_.find("/motor2/backward_limit");
                    if(search == limits_.end())
                    {
                        ROS_WARN("Backward limit for ID2 is empty");
                        break;
                    }

                    std::ofstream fd;
                    fd.open(output_path_);
                    if(!fd.is_open())
                    {
                        ROS_ERROR("Error openning the output file");
                        break;
                    }
                    char buffer[50];
                    for(auto it = limits_.begin(); it != limits_.end(); ++it)
                    {
                        snprintf(buffer, 50, "%s: %d\n", it->first.c_str(), it->second);
                        fd << buffer;
                    }
                    ROS_INFO("Configuration file is successfully created.");
                    fd.close();
                    break;
                }
                default:
                    break;
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        
        boost::shared_ptr<dynamixel_class::DynamixelClass> dynamixel_states_;

        std_msgs::Float64MultiArray dynamixel_command_;

        char c;
        double increment = 1; //unit mm
        std::string key_;
        std::map<std::string, int> limits_;
        std::string output_path_;
    };
}

struct termios old_term;

void signalHandler(int sig)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    exit(EXIT_SUCCESS);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_limits_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    dynamixel_limits::DynamixelLimits dl(nh);

    std::stringstream ss;
    ss << "Keyboard Control Schema: \n" \
       << "ID1, Forward/Backward: f/g \n" \
       << "ID2, Forward/Backward: v/b \n";
    std::string message = ss.str();
    std::cout << message;

    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    old_term = term;
    term.c_lflag &= ~ICANON;
    term.c_lflag &= ~ECHO;
    term.c_cc[VMIN] = 0;
    term.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);

    signal(SIGINT, signalHandler);

    while(ros::ok())
    {
        ros::spinOnce();
        dl.run();
        loop_rate.sleep();
    }
    
    return 0;
}
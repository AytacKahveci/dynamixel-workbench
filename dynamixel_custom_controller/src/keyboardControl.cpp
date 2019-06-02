/* @author Aytaç Kahveci */
#include "ros/ros.h"
#include "dynamixel_custom_controller/dynamixel_class.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>

#define PI 3.1415

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DynamixelKeyboardControl");
    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Publisher state_publisher_;
    ros::Rate loop_rate(200);

    std_msgs::Float64MultiArray commandMotor_;
    std_msgs::Float64 state_;

    commandMotor_.data.resize(4);

    pub_ = nh.advertise<std_msgs::Float64MultiArray>("/dynamixel/JointPosGroup/poseffCommand",1);
    dynamixel_class::DynamixelClass motor(nh, 2);
    ros::spinOnce();
    std::string buffer;
    std::string delim = ",";
    std::string tokens[2];
    int loop = 0;
    bool firsAlt = true;
    double m1;
    double m2;
    while(ros::ok())
    {
        std::cout << "Enter motor1 pos"<< std::endl;
        std::getline(std::cin, buffer);
        m1 = atof(buffer.c_str());
        std::cout << "Enter motor2 pos"<< std::endl;
        std::getline(std::cin, buffer);
        m2 = atof(buffer.c_str());
        std::cout << "Motor1:"<< m1 << " Motor2:"<< m2<< std::endl;

        ros::spinOnce();
        double init = motor.position[0];
        double init2 = motor.position[1];
        ROS_INFO("init data is %f",init);
        ROS_INFO("init2 data is %f",init2);


        commandMotor_.data[0] = init + m1*78.615;
        commandMotor_.data[1] = init2 + m2*76.92;
        commandMotor_.data[3] = 5;
        commandMotor_.data[4] = 5;

        pub_.publish(commandMotor_);
        ROS_INFO("Sended data is %f",commandMotor_.data[0]);
        ROS_INFO("Sended data is %f",commandMotor_.data[1]);
        loop_rate.sleep();
    }
}

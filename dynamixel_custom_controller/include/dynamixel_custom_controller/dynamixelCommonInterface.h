#ifndef DYNAMIXEL_COMMON_INTERFACE_H
#define DYNAMIXEL_COMMON_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/poseff_command_interface.h>
#include <control_toolbox/pid.h>
#include <list>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

namespace dynamixel_common_interface
{
    class DynamixelCommonInterface : public hardware_interface::RobotHW
    {
    public:

        DynamixelCommonInterface(ros::NodeHandle &nh);
        virtual ~DynamixelCommonInterface();
        void write();
        void read();

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PosEffJointInterface jnt_poseff_interface;
        std::string node_ns_;

        double* pos_;
        double* eff_;
        double* vel_;
        double* cmd_pos_;
        double* cmd_eff_;
        double* last_cmd_pos_;
        double* last_cmd_eff_;
        int* limits_;

        std::vector<std::string> motor_names_;
        std::vector<std::string> joint_names_;
        int motorNumbers;
        int jointNumbers;

        std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
        dynamixel_multi_driver::DynamixelMultiDriver* multi_driver_;
        std::vector<int8_t> motorId;
        std::vector<uint8_t> operatingMode;
        std::vector<uint16_t> positionKp;
        std::vector<uint16_t> positionKi;
        std::vector<uint16_t> positionKd;
        std::vector<uint16_t> velocityKp;
        std::vector<uint16_t> velocityKi;
        std::vector<uint16_t> currentLimit;
        std::vector<uint32_t> accelerationLimit;
        std::vector<uint32_t> velocityLimit;
        std::vector<uint32_t> profileVelocity;
        std::vector<uint32_t> profileAcceleration;


        std::string port;
        int baudrate;
        float protocolVersion;

    public:
        typedef struct{
            std::vector<uint8_t> torque;
            std::vector<int16_t> current;
        }WriteValue;

        typedef struct{
            std::vector<uint32_t> cur_pos;
            std::vector<uint32_t> des_pos;
        }MotorPos;

    private:
        bool loadDynamixel();
        void checkLoadDynamixel();
        bool setTorque(bool);
        WriteValue *writeValue_;
        MotorPos *motorPos_;

    public:
        bool setCurrent(double *currentArray);
        bool setPosition(double *currentArray);
        bool setOperatingMode(const std::vector<uint8_t> &mode);
        bool setVelocityProfile(std::vector<uint32_t> &vel);
        bool setAccelerationProfile(std::vector<uint32_t> &acc);
        bool readDynamixelState();

    };
}

#endif

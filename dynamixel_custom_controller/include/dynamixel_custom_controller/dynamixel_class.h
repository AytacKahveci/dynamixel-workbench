#ifndef DYNAMIXEL_CLASS_H
#define DYNAMIXEL_CLASS_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace dynamixel_class
{
    class DynamixelClass
    {
    public:
        DynamixelClass(){}
        
        DynamixelClass(ros::NodeHandle& nh, const int sz)
        {
            position.resize(sz);
            sub = nh.subscribe("/dynamixel/joint_states",1, &DynamixelClass::subCb, this);
        }

        ~DynamixelClass()
        {
            sub.shutdown();
        }

        void subCb(const sensor_msgs::JointState &pos)
        {
            position = pos.position;
        }

    std::vector<double> position;

    private:
        ros::Subscriber sub;
    };
}
#endif

/* @author Aytaç Kahveci */
#include <ros/ros.h>
#include <ros/master.h>
#include <iostream>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <string>
#include <dynamixel_custom_controller/dynamixelCommonInterface.h>

using namespace dynamixel_common_interface;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"dynamixelCommonController");
    ros::Time::init();

    ros::NodeHandle n("/dynamixel");
    ros::NodeHandle nh("~");
    ros::CallbackQueue queue;
	nh.setCallbackQueue(&queue);
	n.setCallbackQueue(&queue);

	ros::AsyncSpinner spinner(4, &queue);
	spinner.start();

    DynamixelCommonInterface robot(n);

    controller_manager::ControllerManager cm(&robot, n);
	ros::Time ts = ros::Time::now();
    ros::Rate loop_rate(200);

    sleep(1);

    while(ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
		robot.read();
		ts = ros::Time::now();
		cm.update(ts, d);
		robot.write();
		loop_rate.sleep();
    }

    spinner.stop();

	return 0;
}

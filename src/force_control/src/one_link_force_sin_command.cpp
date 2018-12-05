#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

using namespace std;

void pub_thread()
{
    ros::NodeHandle n;
    ros::Publisher force_pub = n.advertise<std_msgs::Float64>("one_link/joint1_effort_controller/command", 100);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        std_msgs::Float64 msg;
        msg.data = -2*9.80000000000001*0.5*sin(ros::Time::now().toSec());
        force_pub.publish(msg);
        loop_rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "force_control");
    ros::NodeHandle nh_;

    std::thread th1(pub_thread);
    th1.join();

	return 0;
}

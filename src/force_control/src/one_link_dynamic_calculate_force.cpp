#include <iostream>
#include <thread>
#include <math.h>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "filter/LowPassFilter.hpp"

using namespace std;

double position,velocity_pre,velocity_now,acc_no_filter,acc_filter,force_real;
std::chrono::high_resolution_clock::time_point t_pre,t_new;

void Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    position = msg->position[0];
    if(msg->velocity.size()>0)
    {
        t_new = std::chrono::high_resolution_clock::now();
        velocity_now = msg->velocity[0];
        std::chrono::duration<double, std::milli> fp_ms = t_new - t_pre;
        double time_step = fp_ms.count();
        double acc_no_filter_temp = (velocity_now - velocity_pre)/(time_step/1000);

		acc_no_filter = acc_no_filter_temp;
        LowPassFilter lpf(50,0.5);
        acc_filter = lpf.update(acc_no_filter);
    }

    if(msg->effort.size()>0)
    {
        force_real = msg->effort[0];
    }

    velocity_pre = velocity_now;
    t_pre = t_new;
}

void pub_acc_thread()
{
    ros::NodeHandle n;
    ros::Publisher acc_filter_pub = n.advertise<std_msgs::Float64>("acc_filter", 100);
    ros::Publisher compute_force_pub = n.advertise<std_msgs::Float64>("compute_force", 100);
    ros::Publisher force_error_pub = n.advertise<std_msgs::Float64>("force_error", 100);
    ros::Publisher acc_no_filter_pub = n.advertise<std_msgs::Float64>("acc_no_filter", 100);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        std_msgs::Float64 msg_filter,msg_no_filter;
        msg_filter.data = acc_filter;
        msg_no_filter.data = acc_no_filter;
        if(fabs(acc_filter)>10)
            continue;
        acc_filter_pub.publish(msg_filter);
        acc_no_filter_pub.publish(msg_no_filter);

        std_msgs::Float64 msg_compute_force,msg_force_error;
        double compute_force_temp = 4*9.8*0.5*sin(position) + 4*0.5*0.5*acc_filter;
        if(fabs(compute_force_temp)>=15)
            continue;
        msg_compute_force.data = compute_force_temp;
        compute_force_pub.publish(msg_compute_force);

        msg_force_error.data = (force_real-compute_force_temp);
        force_error_pub.publish(msg_force_error);

        loop_rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "dynamic_calculate");
    ros::NodeHandle nh_;

    t_pre = std::chrono::high_resolution_clock::now();
    ros::Subscriber sub = nh_.subscribe("joint_states", 100, Callback);

    std::thread th1(pub_acc_thread);

    ros::spin();
    th1.join();

	return 0;
}

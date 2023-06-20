/**
**  Node Pendulum Controller
    Subscribers: "control_input" string
    Publishers: "control_output" geometry_msgs/Wrench
**/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Wrench.h>

#define PI 3.14159265

class controller
{
private:

    //Subscribers

    ros::Subscriber control_input;

    //Publishers

    ros::Publisher control_output;

    // try using ROS time

    // ros::Time time_begin = ros::Time::now();

    // ros::Time time_end = ros::Time::now();

    float PID_input;

    float PID_input_prev;

    float dt = 0.1;

    float kp = 20, ki = 5, kd = 0.01;

    float torque;

    float angle_input;
    

public:
    controller(ros::NodeHandle *n)
    {

        control_input = n->subscribe("c_input", 1000, &controller::c_input,this);

        control_output = n->advertise<geometry_msgs::Wrench>("c_output", 1000);

    }

    void c_input(const std_msgs::String::ConstPtr& msg) // recieve actual angle
    {
        controller::angle_input = std::stof(msg->data);

        ROS_INFO_STREAM("Actual Angle Value: "<< (180*controller::angle_input/PI));

        controller::PID_input = std::stof(msg->data);
    }

    void c_output()
    {

        ros::Rate rate(10);

        while (ros::ok())
        {   
            // controller::time_end = ros::Time::now();

            // controller::dt = abs((controller::time_end - controller::time_begin).toNSec()*1e-1);

            controller::torque = controller::kp*(controller::PID_input) +                                              //   P
                                 controller::ki*(controller::PID_input)*(controller::dt) +                             //   I   controller
                                 controller::kd*(controller::PID_input - controller::PID_input_prev)/(controller::dt); //   D

            geometry_msgs::Wrench msg;

            msg.torque.x = controller::torque;


            control_output.publish(msg);

            controller::PID_input_prev = controller::PID_input;

            // controller::time_begin = controller::time_end;

            ros::spinOnce();

            rate.sleep();
        }
    }
    
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node_pendulum_controller");

    ros::NodeHandle n;

    controller control = controller(&n);

    control.c_output();

    ros::spin();

    return 0;
}
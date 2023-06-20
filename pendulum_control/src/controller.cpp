/**
**  Node Pendulum Controller
    Subscribers: "control_input" string
    Publishers: "control_output" geometry_msgs/Wrench
**/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Wrench.h>

class controller
{
private:

    double PID_input;

    double PID_input_prev = 0;

    ros::Subscriber control_input;

    ros::Publisher control_output;

    

public:
    controller(ros::NodeHandle *n)
    {

        control_input = n->subscribe("c_input", 1000, &controller::c_input,this);

        control_output = n->advertise<geometry_msgs::Wrench>("c_output", 1000);

    }

    void c_input(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());

        controller::PID_input = std::stod(msg->data);
    }

    void c_output()
    {

        ros::Rate rate(10);

        while (ros::ok())
        {   
            float kp = 0.3, kd = 2;

            float torque = kp*(controller::PID_input) + kd*(controller::PID_input - controller::PID_input_prev);

            geometry_msgs::Wrench msg;

            msg.force.x = 10;
            msg.torque.x = torque;

            control_output.publish(msg);

            controller::PID_input_prev = controller::PID_input;

            ros::spinOnce();

            // count = count + 1;

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


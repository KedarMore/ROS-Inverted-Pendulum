/**
**  Node Pendulum Simulator
    Publishers: "control_input" string
                "pendulum_viz"  vizualisation_msgs/MarkerArray
    Subscribers: "control_output" geometry_msgs/Wrench
**/
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Wrench.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#define PI 3.14159265

class simulator
{
private:

    //Subscribers

    ros::Subscriber control_output;

    //Publishers

    ros::Publisher control_input;

    ros::Publisher pendulum_viz;

    float torque = 0;

    float dt = 0.1;

    float angle_prev;

public:
    
    simulator(ros::NodeHandle *n , int x)
    {

        control_output = n->subscribe("c_output", 1000, &simulator::c_output,this);

        control_input = n->advertise<std_msgs::String>("c_input", 1000);

        pendulum_viz = n->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);

        simulator::angle_prev = atan2(sin(PI*x/180),cos(PI*x/180)); // Put any angle between -PI and PI

    }

    void c_output(const geometry_msgs::Wrench& msg) // revieve torque values
    {
        ROS_INFO_STREAM("Motor Torque Value: "<<msg.torque.x);

        simulator::torque = msg.torque.x;
    }

    void pub() // control_input and pendulum_viz
    {
        ros::Rate rate(10);

        while (ros::ok())
        {
            // control_input

            std_msgs::String trial;

            std::stringstream ss;

            angle_prev = angle_prev - simulator::torque*(simulator::dt)*(simulator::dt);

            ss << (angle_prev);
            trial.data = ss.str();

            control_input.publish(trial); // send actual angle

            
            // pendulum_viz

            visualization_msgs::MarkerArray mass;

            int num_circ = 2;
            mass.markers.resize(num_circ);

            uint32_t shape [] = {visualization_msgs::Marker::CYLINDER,visualization_msgs::Marker::SPHERE}; // marker array

            for (int i = 0; i < num_circ; i++)
            {

                if (i == 0)
                {
                    mass.markers[i].header.frame_id = "map";
                    mass.markers[i].header.stamp = ros::Time::now();

                    mass.markers[i].ns = "node_pendulum_simulator";
                    mass.markers[i].id = i;

                    mass.markers[i].type = shape[i];

                    mass.markers[i].action = visualization_msgs::Marker::ADD;

                    mass.markers[i].pose.position.x = 2.5*sin(angle_prev);
                    mass.markers[i].pose.position.y = 0;
                    mass.markers[i].pose.position.z = 2.5*cos(angle_prev);

                    tf2::Quaternion myQuaternion;

                    myQuaternion.setRPY(0,angle_prev,0); //x,y,z

                    myQuaternion=myQuaternion.normalize();

                    mass.markers[i].pose.orientation.x = myQuaternion.getX();
                    mass.markers[i].pose.orientation.y = myQuaternion.getY();
                    mass.markers[i].pose.orientation.z = myQuaternion.getZ();
                    mass.markers[i].pose.orientation.w = myQuaternion.getW();

                    mass.markers[i].scale.x = 0.1;
                    mass.markers[i].scale.y = 0.1;
                    mass.markers[i].scale.z = 5.0;

                    mass.markers[i].color.r = 0.0f;
                    mass.markers[i].color.g = 1.0f;
                    mass.markers[i].color.b = 0.0f;
                    mass.markers[i].color.a = 1.0;
                }
                else
                {
                    mass.markers[i].header.frame_id = "map";
                    mass.markers[i].header.stamp = ros::Time::now();

                    mass.markers[i].ns = "node_pendulum_simulator";
                    mass.markers[i].id = i;

                    mass.markers[i].type = shape[i];

                    mass.markers[i].action = visualization_msgs::Marker::ADD;

                    mass.markers[i].pose.position.x = 5*sin(angle_prev);
                    mass.markers[i].pose.position.y = 0;
                    mass.markers[i].pose.position.z = 5*cos(angle_prev);
                    mass.markers[i].pose.orientation.x = 0.0;
                    mass.markers[i].pose.orientation.y = 0.0;
                    mass.markers[i].pose.orientation.z = 0.0;
                    mass.markers[i].pose.orientation.w = 1.0;

                    mass.markers[i].scale.x = 1.0;
                    mass.markers[i].scale.y = 1.0;
                    mass.markers[i].scale.z = 1.0;

                    mass.markers[i].color.r = 0.0f;
                    mass.markers[i].color.g = 1.0f;
                    mass.markers[i].color.b = 0.0f;
                    mass.markers[i].color.a = 1.0;
                }

            }

            pendulum_viz.publish(mass); // send information to rviz

            ros::spinOnce();

            rate.sleep();

        }
        
    }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node_pendulum_simulator");

    ros::NodeHandle n;

    int x; 

    std::cout << "Select initial pendulum angle: ";

    std::cin >> x;

    simulator simulate = simulator(&n, x);

    simulate.pub();

    ros::spin();

    return 0;
}
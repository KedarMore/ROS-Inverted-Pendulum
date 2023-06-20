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

#define PI 3.14159265

class simulator
{
private:

    int count = 0;

    float torque;

    float angle_prev = 50;

    ros::Subscriber control_output;

    ros::Publisher control_input;

    ros::Publisher pendulum_viz;

public:
    
    simulator(ros::NodeHandle *n)
    {

        control_output = n->subscribe("c_output", 1000, &simulator::c_output,this);

        control_input = n->advertise<std_msgs::String>("c_input", 1000);

        pendulum_viz = n->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);

    }

    void c_output(const geometry_msgs::Wrench& msg)
    {
        ROS_INFO_STREAM("I heard back:"<<msg.torque.x);

        simulator::torque = msg.torque.x;
    }

    void pub()
    {
        ros::Rate rate(10);

        while (ros::ok())
        {

            std_msgs::String trial;

            std::stringstream ss;

            simulator::angle_prev = simulator::angle_prev-simulator::torque;

            ss << (simulator::angle_prev);
            trial.data = ss.str();
            // trial.data = (simulator::angle_prev-simulator::torque);

            control_input.publish(trial);

            


            visualization_msgs::MarkerArray mass;

            int num_circ = 2;
            mass.markers.resize(num_circ);

            // visualization_msgs::Marker rod;
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.

            uint32_t shape [] = {visualization_msgs::Marker::CYLINDER,visualization_msgs::Marker::SPHERE};

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

                    mass.markers[i].pose.position.x = 2.5*sin(0.1*simulator::angle_prev);
                    mass.markers[i].pose.position.y = 0;
                    mass.markers[i].pose.position.z = 2.5*cos(0.1*simulator::angle_prev);

                    tf2::Quaternion myQuaternion;

                    myQuaternion.setRPY(0,0.1*simulator::angle_prev,0); //x,y,z

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

                    mass.markers[i].pose.position.x = 5*sin(0.1*simulator::angle_prev);
                    mass.markers[i].pose.position.y = 0;
                    mass.markers[i].pose.position.z = 5*cos(0.1*simulator::angle_prev);
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

            simulator::count++;

            pendulum_viz.publish(mass);

            ros::spinOnce();

            rate.sleep();

        }
        
    }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node_pendulum_simulator");

    ros::NodeHandle n;

    simulator simulate = simulator(&n);

    simulate.pub();

    ros::spin();

    return 0;
}






































// void c_output(const geometry_msgs::Wrench& msg)
// {

//     ROS_INFO_STREAM("I heard back:"<<msg.torque.x);
// }

// int main(int argc, char* argv[])
// {
//     ros::init(argc, argv, "node_pendulum_simulator");

//     ros::NodeHandle n;

//     ros::Publisher control_input = n.advertise<std_msgs::String>("c_input", 1000);

//     ros::Publisher pendulum_viz = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);

//     ros::Subscriber control_output = n.subscribe("c_output", 1000, c_output);

//     ros::Rate rate(10);

//     int count = 0;

//     while (ros::ok())
//     {
//         std_msgs::String trial;

//         // std::stringstream ss;
//         // ss << msg.torque.x;
//         trial.data = "11";

//         control_input.publish(trial);

//         ros::spinOnce();

//         rate.sleep();

//         visualization_msgs::MarkerArray mass;

//         int num_circ = 2;
//         mass.markers.resize(num_circ);

//         // visualization_msgs::Marker rod;
//         // Set the frame ID and timestamp.  See the TF tutorials for information on these.

//         uint32_t shape [] = {visualization_msgs::Marker::CYLINDER,visualization_msgs::Marker::SPHERE};

//         for (int i = 0; i < num_circ; i++)
//         {

//             if (i == 0)
//             {
//                 mass.markers[i].header.frame_id = "map";
//                 mass.markers[i].header.stamp = ros::Time::now();

//                 mass.markers[i].ns = "node_pendulum_simulator";
//                 mass.markers[i].id = i;

//                 mass.markers[i].type = shape[i];

//                 mass.markers[i].action = visualization_msgs::Marker::ADD;

//                 mass.markers[i].pose.position.x = 2.5*sin(0.1*count);
//                 mass.markers[i].pose.position.y = 0;
//                 mass.markers[i].pose.position.z = 2.5*cos(0.1*count);

//                 tf2::Quaternion myQuaternion;

//                 myQuaternion.setRPY(0,0.1*count,0); //x,y,z

//                 myQuaternion=myQuaternion.normalize();

//                 mass.markers[i].pose.orientation.x = myQuaternion.getX();
//                 mass.markers[i].pose.orientation.y = myQuaternion.getY();
//                 mass.markers[i].pose.orientation.z = myQuaternion.getZ();
//                 mass.markers[i].pose.orientation.w = myQuaternion.getW();

//                 mass.markers[i].scale.x = 0.1;
//                 mass.markers[i].scale.y = 0.1;
//                 mass.markers[i].scale.z = 5.0;

//                 mass.markers[i].color.r = 0.0f;
//                 mass.markers[i].color.g = 1.0f;
//                 mass.markers[i].color.b = 0.0f;
//                 mass.markers[i].color.a = 1.0;
//             }
//             else
//             {
//                 mass.markers[i].header.frame_id = "map";
//                 mass.markers[i].header.stamp = ros::Time::now();

//                 mass.markers[i].ns = "node_pendulum_simulator";
//                 mass.markers[i].id = i;

//                 mass.markers[i].type = shape[i];

//                 mass.markers[i].action = visualization_msgs::Marker::ADD;

//                 mass.markers[i].pose.position.x = 5*sin(0.1*count);
//                 mass.markers[i].pose.position.y = 0;
//                 mass.markers[i].pose.position.z = 5*cos(0.1*count);
//                 mass.markers[i].pose.orientation.x = 0.0;
//                 mass.markers[i].pose.orientation.y = 0.0;
//                 mass.markers[i].pose.orientation.z = 0.0;
//                 mass.markers[i].pose.orientation.w = 1.0;

//                 mass.markers[i].scale.x = 1.0;
//                 mass.markers[i].scale.y = 1.0;
//                 mass.markers[i].scale.z = 1.0;

//                 mass.markers[i].color.r = 0.0f;
//                 mass.markers[i].color.g = 1.0f;
//                 mass.markers[i].color.b = 0.0f;
//                 mass.markers[i].color.a = 1.0;
//             }

//         }

//         count++;

//         pendulum_viz.publish(mass);

//     }
// }
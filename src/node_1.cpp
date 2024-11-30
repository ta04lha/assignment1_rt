#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>





void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
	{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",  
	msg->x, msg->y, msg->theta);
	
	}
	
	
	
	
int main (int argc, char **argv)
{
	ros::init(argc, argv, "node_1");  
	ros::NodeHandle nh;


        ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv1;
	srv1.request.x = 1.0;
	srv1.request.y = 2.0;
 	srv1.request.theta = 0.0;
 	srv1.request.name = "turtle2";
 	client1.call(srv1);
        
        
       ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
       ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
 
 
       ros::Rate loop rate(10);
       std::string turtle_name;
       
       double linear_velocity, angular_velocity ;
   
      while (ros::ok())
      {
      std::cout << "enter the turtle name (turtle1 or turtle2):          "  ;
      std::cin >> turtle_name;
      std::cout << "enter the turtle linear velocity:   " ;
      std::cin >> linear_velocity
      std::cout << "enter the turtle angular velocity:   " ;
      std::cin >> angular_velocity
      
      
      
      
      ros::Publisher selected_pub = (turtle_name == "turtle1") ? pub_turtle1 : pub_turtle2;
      
      geometry_msgs::Twist vel_msg;
      vel_msg.linear.x = linear_velocity;
      vel_msg.angular.z = angular_velocity;
      
      
      
      selected_pub.publish(vel_msg);
      ros::Duration(1.0).sleep();
      
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      selected_pub.publish(vel_msg);
      
      
      ros::spinOnce();
      loop rate.sleep();
      
      }
    return 0;
    
    }  
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      

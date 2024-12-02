#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath> 







const double DISTANCE_THRESHOLD = 2;
const double BOUNDARY_THRESHOLD = 1;


ros::Publisher pub_turtle1;
ros::Publisher pub_turtle2;


void poseCallbackturtle1(const turtlesim::Pose::ConstPtr& msg)

{  
       double x1 = msg->x;
       double y1 = msg->y;
       
       
       static double x2 = 0.0;
       static double y2 = 0.0;
       
       
       
       double distance = sqrt(pow(x2-x1 ,2)+pow(y2-y1 ,2));
       
       
       
       std_msgs::Float32 distance_msg;
       distance_msg.data = distance;
       ros::NodeHandle nh;
       ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
       distance_pub.publish(distance_msg);
       
       ROS_INFO("distance between turtle1 and turtle2:f%", distance);
       
       
        if (distance < DISTANCE_THRESHOLD || x1 < BOUNDARY_THRESHOLD || x1 > 10.0 || y1 < BOUNDARY_THRESHOLD || y1 > 10.0)
    {
        
        geometry_msgs::Twist stop_msg;
        pub_turtle1.publish(stop_msg);
        ROS_INFO("Turtle1 stopped due to distance or boundary");
        
       
      }
        
        
  } 
        
        
void poseCallbackturtle2(const turtlesim::Pose::ConstPtr& msg)

{  
       double x2 = msg->x;
       double y2 = msg->y;
       
       
       static double x1 = 0.0;
       static double y1 = 0.0;
       
       
       
       double distance = sqrt(pow(x2-x1 ,2)+pow(y2-y1 ,2));
       
       
       
       std_msgs::Float32 distance_msg;
       distance_msg.data = distance;
       ros::NodeHandle nh;
       ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
       distance_pub.publish(distance_msg);
       
       ROS_INFO("distance between turtle1 and turtle2:f%", distance);
       
       
       if (distance < DISTANCE_THRESHOLD || x2 < BOUNDARY_THRESHOLD || x2 > 10.0 || y2 < BOUNDARY_THRESHOLD || y2 > 10.0)
    {
        
        geometry_msgs::Twist stop_msg;
        pub_turtle2.publish(stop_msg);
        ROS_INFO("Turtle2 stopped due to distance or boundary");
        
       
      }
        
        
  } 
       
       
       
int main(int argc, char **argv)
{

        ros::init(argc,argv, "node_2");
        ros::NodeHandle nh;
        
        
        
        
        pub_turtle1 = nh.advertise<geometry_msgs::Twist> ("/turtle1/cmd_vel", 10);
        pub_turtle2 = nh.advertise<geometry_msgs::Twist> ("/turtle2/cmd_vel", 10);
        
        
        
        
        ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, poseCallbackturtle1);
        ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, poseCallbackturtle2);


  
        ros::spin();
        
        return 0;









}
        
        
        
        
        
        
        
        
        
        
       

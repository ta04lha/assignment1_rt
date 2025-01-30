#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "turtlesim/TeleportAbsolute.h"
#include <cmath>
#include <cstdlib>

const double DISTANCE_THRESHOLD = 2.0;
const double BOUNDARY_THRESHOLD = 1.0;
const double SAFE_DISTANCE = 2.5;

ros::Publisher pub_turtle1;
ros::Publisher pub_turtle2;
ros::ServiceClient teleport_turtle1;
ros::ServiceClient teleport_turtle2;

double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;

double getRandomCoordinate(double min, double max) {
    return min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (max - min)));
}

void teleportTurtle(const std::string& turtle_name, ros::ServiceClient& teleport_client) {
    turtlesim::TeleportAbsolute srv;
    srv.request.x = getRandomCoordinate(BOUNDARY_THRESHOLD + SAFE_DISTANCE, 10.0 - SAFE_DISTANCE);
    srv.request.y = getRandomCoordinate(BOUNDARY_THRESHOLD + SAFE_DISTANCE, 10.0 - SAFE_DISTANCE);
    srv.request.theta = 0.0;
    
    if (teleport_client.call(srv)) {
        ROS_INFO("%s teleported to safe position: (%.2f, %.2f)", turtle_name.c_str(), srv.request.x, srv.request.y);
    } else {
        ROS_ERROR("Failed to teleport %s", turtle_name.c_str());
    }
}

void poseCallbackTurtle1(const turtlesim::Pose::ConstPtr& msg) {
    x1 = msg->x;
    y1 = msg->y;
    
    double distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    
    if (distance < DISTANCE_THRESHOLD || x1 < BOUNDARY_THRESHOLD || x1 > 10.0 || y1 < BOUNDARY_THRESHOLD || y1 > 10.0) {
        teleportTurtle("turtle1", teleport_turtle1);
    }
}

void poseCallbackTurtle2(const turtlesim::Pose::ConstPtr& msg) {
    x2 = msg->x;
    y2 = msg->y;
    
    double distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    
    if (distance < DISTANCE_THRESHOLD || x2 < BOUNDARY_THRESHOLD || x2 > 10.0 || y2 < BOUNDARY_THRESHOLD || y2 > 10.0) {
        teleportTurtle("turtle2", teleport_turtle2);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_teleport_node");
    ros::NodeHandle nh;
    
    pub_turtle1 = nh.advertise<geometry_msgs::Twist> ("/turtle1/cmd_vel", 10);
    pub_turtle2 = nh.advertise<geometry_msgs::Twist> ("/turtle2/cmd_vel", 10);
    
    teleport_turtle1 = nh.serviceClient<turtlesim::TeleportAbsolute> ("/turtle1/teleport_absolute");
    teleport_turtle2 = nh.serviceClient<turtlesim::TeleportAbsolute> ("/turtle2/teleport_absolute");
    
    ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, poseCallbackTurtle1);
    ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, poseCallbackTurtle2);
    
    ros::spin();
    return 0;
}

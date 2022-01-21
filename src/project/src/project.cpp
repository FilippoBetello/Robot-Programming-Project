#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "message_filters/subscriber.h" 


float vel_x = 0, vel_y = 0;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    vel_x = msg->linear.x;
    vel_y = msg->linear.y;
    std::cout << vel_x << " Velocity x" << std::endl;
    std::cout << vel_y << " Velocity y" << std::endl;
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  ROS_INFO("Laser_scan: [%s]", msg->header.frame_id.c_str());
}

int main(int argc, char **argv){

  ros::init(argc, argv, "project");

  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);
  ros::Subscriber laser_scan_sub = nh.subscribe("base_scan", 1000, laser_cmd_vel_callback);
  ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
 /* Exit only when ctrl+c is pressed*/
  ros::spin();

  return 0;
} 
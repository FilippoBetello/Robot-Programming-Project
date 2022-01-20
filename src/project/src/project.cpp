// I've done a mess
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>


void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  ROS_INFO("Laser_scan: [%s]", msg->header.frame_id.c_str());
}

int main(int argc, char **argv){

  ros::init(argc, argv, "project");

  ros::NodeHandle nh;

  ros::Subscriber laser_scan_sub = nh.subscribe("base_scan", 1000, laser_cmd_vel_callback);

 /* Exit only when ctrl+c is pressed*/
  ros::spin();

} 
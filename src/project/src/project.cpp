#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "message_filters/subscriber.h" 
#include "tf/message_filter.h"
#include <Eigen/Geometry>

float vel_x = 0, vel_y = 0, obstacle_x = 0, obstacle_y = 0;


inline Eigen::Isometry2f convertPose2D(const tf::StampedTransform& t) {
    double yaw,pitch,roll;
    tf::Matrix3x3 mat =  t.getBasis();
    mat.getRPY(roll, pitch, yaw);
    Eigen::Isometry2f T;
    T.setIdentity();
    Eigen::Matrix2f R;
    R << std::cos(yaw), -std::sin(yaw),
        std::sin(yaw), std::cos(yaw);
    T.linear() = R;
    T.translation() = Eigen::Vector2f(t.getOrigin().x(), t.getOrigin().y());
    return T;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    vel_x = msg->linear.x;
    vel_y = msg->linear.y;
    std::cout << vel_x << " Velocity x" << std::endl;
    std::cout << vel_y << " Velocity y" << std::endl;
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& scan){

  laser_geometry::LaserProjection projector;
  tf::StampedTransform obstacle;
  tf::TransformListener listener;
  sensor_msgs::PointCloud cloud;
  try{
    // Transform laser scan in odom frame using the tf listener
    projector.transformLaserScanToPointCloud("base_laser_link",*scan, cloud,listener);
    listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10,0));
    listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), obstacle);

    Eigen::Isometry2f T = convertPose2D(obstacle);  //Matrix to transform the coordinates
    Eigen::Vector2f p;

    std::cout << "ostacoli x: " << obstacle_x << std::endl;
    std::cout << "ostacoli y: " << obstacle_y << std::endl;
  }

  catch(tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
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
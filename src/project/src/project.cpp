#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "message_filters/subscriber.h" 
#include "tf/message_filter.h"
#include <Eigen/Geometry>

float vel_x = 0, vel_y = 0, vel_angular = 0, force_x = 0, force_y = 0;
bool cmd_vel_arrived = false;
geometry_msgs::Twist vel_rec;
ros::Publisher pub_vel;

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
  vel_rec = *msg;
  cmd_vel_arrived = true;
  vel_x = msg->linear.x;
  vel_y = msg->linear.y;
  vel_angular = msg->angular.z;
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
  if (!cmd_vel_arrived) return;
  cmd_vel_arrived = false;    //is arrived, if it is not set to false the same command will be processed multiple times
  
  laser_geometry::LaserProjection projector;
  tf::StampedTransform obstacle;
  tf::TransformListener listener;
  sensor_msgs::PointCloud cloud;

  try{
    // Transform laser scan in odom frame using the tf listener
    projector.transformLaserScanToPointCloud("base_laser_link",*scan, cloud,listener);
    listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10,0));
    listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), obstacle);
  }

  catch(tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Isometry2f T = convertPose2D(obstacle);  //Matrix to transform the coordinates
  Eigen::Vector2f p_start, p_curr;

  //at the beginning 
  p_start(0) = cloud.points[540].x;
  p_start(1) = cloud.points[540].y;
  p_start = T*p_start;
  float distance_obstacle = sqrt(pow(p_start(0),2) + pow(p_start(1),2));


//scan all elements of the cloud
  for(auto& point: cloud.points){
    p_curr(0) = point.x;
    p_curr(1) = point.y;
    p_curr = T*p_curr;
    float distance_obstacle_curr = sqrt(point.x*point.x + point.y*point.y);   //will be substituted by pow, now for me it's clearer to work like this
    float force_obstacle = 1/pow(distance_obstacle_curr, 2);
    //update components x y
    force_x += p_start(0)*force_obstacle;
    force_y += p_start(1)*force_obstacle;

    if(distance_obstacle_curr < distance_obstacle){
      distance_obstacle = distance_obstacle_curr;
      p_start = p_curr;
    }
  }

  force_x = - force_x;
  force_y = - force_y;
  geometry_msgs::Twist msg_send;

  //coming near to the obstacle
  if (distance_obstacle < 0.2){
    ROS_INFO("DISTANCE IS: %f", distance_obstacle);
   
    msg_send.linear.x =  (force_x + vel_x)/2500;
    msg_send.linear.y =  (force_y + vel_y)/2500;

    if(p_start(1) > 0){
      msg_send.angular.z = -1/distance_obstacle;
    }
    else if (p_start(1) < 0){
      msg_send.angular.z = 1/distance_obstacle;
    }
    pub_vel.publish(msg_send);
    ROS_INFO("Rotate of: %f", msg_send.angular.z);
  }

 /*//very close to the obstacle
  else if(distance_obstacle <= 0.1 && vel_x>0){
    ROS_INFO("DISTANCE IS: %f", distance_obstacle);
    //decrease by a bigger factor the velocities
    msg_send.linear.x =  (force_x + vel_x)/2000;
    msg_send.linear.y =  (force_y + vel_y)/2000;

    if(p_start(1) > 0){
      msg_send.angular.z = -1/distance_obstacle;
    }
    else if (p_start(1) < 0){
      msg_send.angular.z = 1/distance_obstacle;
    }
    pub_vel.publish(msg_send);
    ROS_INFO("Second, velocities are: %f", msg_send.linear.x);
  }
 /* //too much close, the robot stops and turn around
  else if(distance_obstacle <= 0.1){
    ROS_INFO("DISTANCE IS: %f", distance_obstacle);
    //null velocites otherwise crashes
    msg_send.linear.x =  (force_x + vel_x)/2000;
    msg_send.linear.y =  (force_y + vel_y)/2000;

    if(p_start(1) > 0){
      msg_send.angular.z = -1/distance_obstacle;
    }
    else if (p_start(1) < 0){
      msg_send.angular.z = 1/distance_obstacle;
    }
    pub_vel.publish(msg_send);
    ROS_INFO("Third, velocities are: %f", msg_send.linear.x);
  }*/

  else{
    pub_vel.publish(vel_rec);
    ROS_INFO("DISTANCE IS: %f", distance_obstacle);
  }
  
  
}

int main(int argc, char **argv){

  ros::init(argc, argv, "project");

  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmd_vel_callback);
  ros::Subscriber laser_scan_sub = nh.subscribe("base_scan", 1000, laser_cmd_vel_callback);
  pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  


 /* Exit only when ctrl+c is pressed*/
  ros::spin();

  return 0;
} 
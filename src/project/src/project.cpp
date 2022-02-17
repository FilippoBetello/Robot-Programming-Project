#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "message_filters/subscriber.h" 
#include "tf/message_filter.h"
#include <Eigen/Geometry>

const float INITIAL_DISTANCE = 99999;
const float MAX_DISTANCE = 0.6;
const int DAMPING_FACTOR = 9000;

// global useful variables
float vel_x = 0, vel_y = 0, vel_angular = 0, force_x = 0, force_y = 0;
bool cmd_vel_arrived = false;

geometry_msgs::Twist vel_rec;
ros::Publisher pub_vel;
geometry_msgs::Twist msg_send;

// Transformation
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

// save the vel commands given in input
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
    projector.transformLaserScanToPointCloud("base_laser_link", *scan, cloud,listener);
    listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10,0));
    listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), obstacle);
  }

  catch(tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Isometry2f T = convertPose2D(obstacle);  //Matrix to transform the coordinates
  Eigen::Vector2f p_start, p_curr;

  //Initialize the distance
  float distance_obstacle = INITIAL_DISTANCE;


//scan all elements of the cloud
  for(auto& point: cloud.points){
    p_curr(0) = point.x;
    p_curr(1) = point.y;
    p_curr = T*p_curr;
    float distance_obstacle_curr = sqrt(pow(point.x, 2) + pow(point.y, 2));   
    float force_obstacle = 1/pow(distance_obstacle_curr, 2);
    //update components x y
    force_x += p_curr(0)*force_obstacle;
    force_y += p_curr(1)*force_obstacle;

    if(distance_obstacle_curr < distance_obstacle){
      distance_obstacle = distance_obstacle_curr;
      p_start = p_curr;
    }
  }

  force_x = - force_x;
  force_y = - force_y;

  geometry_msgs::Twist msg_send;

  //coming near to the obstacle
  if (distance_obstacle < MAX_DISTANCE){    //0.6

    std::cout << "DANGER ZONE!! DISTANCE IS: " << distance_obstacle << std::endl;
    force_x = force_x/DAMPING_FACTOR;     //9000
    force_y = force_y/DAMPING_FACTOR;
    
    msg_send.linear.x =  vel_x + force_x;
    msg_send.linear.y =  vel_y + force_y;

    if(p_start(1) > 0){
      msg_send.angular.z = -1/pow(distance_obstacle, 0.7)  - abs(vel_angular);
    }
    else if (p_start(1) < 0){
      msg_send.angular.z = 1/pow(distance_obstacle, 0.7)  + abs(vel_angular);   //counter clockwise
    }
    std::cout << "Velocity: " << msg_send.linear.x << "\tRotation: " << msg_send.angular.z << std::endl;
    pub_vel.publish(msg_send);
  }

  else{
    std::cout << "DISTANCE IS: " << distance_obstacle << std::endl;
    pub_vel.publish(vel_rec);
  }
  
  
}

int main(int argc, char **argv){

  ros::init(argc, argv, "project");

  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmd_vel_callback);
  ros::Subscriber laser_scan_sub = nh.subscribe("base_scan", 8, laser_cmd_vel_callback);

  pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
 /* Exit only when ctrl+c is pressed*/
  ros::spin();

  return 0;
} 
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <tf/transform_datatypes.h>

void chatterIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("EULER Imu Orientation Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);
  ROS_INFO("QUATERNION Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("imu", 1000, chatterIMU);

  ros::spin();

  return 0;
}

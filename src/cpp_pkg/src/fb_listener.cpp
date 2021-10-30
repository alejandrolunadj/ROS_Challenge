#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"


void feedBackCB(const geometry_msgs::Pose2D &feedbackPose){
  ROS_INFO("Feddback position x=%f y=%f", feedbackPose.x, feedbackPose.y);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle node;
  ros::Subscriber sub =  node.subscribe("feedback", 0, feedBackCB,  ros::TransportHints().tcpNoDelay());
  ros::spin();

  return 0;
}

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

using namespace std;

geometry_msgs::Pose2D targetGoal;


void set_targetGoal() {

  cout << "Where do you want the turtle to go?" << endl;
  cout << "Enter target x: ";
  cin >> targetGoal.x;
  cout << "Enter target y: ";
  cin >> targetGoal.y;
  cout << endl;
}

int main(int argc, char **argv) { 

  ros::init(argc, argv, "sender");

  ros::NodeHandle node;

  ros::Publisher targetPub =  node.advertise<geometry_msgs::Pose2D>("target", 0);
  ros::Rate loop_rate(0.5);
  while (ros::ok()) {
    set_targetGoal();
    targetPub.publish(targetGoal);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
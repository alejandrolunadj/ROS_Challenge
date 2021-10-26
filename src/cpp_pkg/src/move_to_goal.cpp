#include <cstdlib>
#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"

using namespace std;

geometry_msgs::Pose2D currentPos;// Current posiition
geometry_msgs::Pose2D targetGoal; // Target GOAL (x, y)
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
ros::Publisher velocityPub; // Object used for publishing velocity command

const double PI = 3.141592654;

const double SF_L = 0.5; //speed factor linear

const double SF_A = 0.5; //speed factor angular

const double distanceTolerance = 0.1;
const double angleTolerance = 0.001; //0.01;
bool isMoving = false;


void setup() {
  targetGoal.x = 5.0;
  targetGoal.y = 5.0;
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}

// Get the distance between the current x,y and the target x,y
double getDistanceToTarget() {
  return sqrt(pow(targetGoal.x - currentPos.x, 2) + pow(targetGoal.y - currentPos.y, 2));
}

// Get the heading error
// i.e. how many radians does the robot need 
// to turn to head towards the waypoint  
double getHeadingError() {

  double deltaX = targetGoal.x - currentPos.x;
  double deltaY = targetGoal.y - currentPos.y;
  double TargetHeading = atan2(deltaY, deltaX);

  double headingError = TargetHeading - currentPos.theta;

  // Make sure heading error falls within -PI to PI range
  if (headingError > PI) {
    headingError = headingError - (2 * PI);
  } 
  if (headingError < -PI) {
    headingError = headingError + (2 * PI);
  }

  return headingError;

}


void setVelocity() {

  double distanceToTarget = getDistanceToTarget();
  double headingError = getHeadingError();

  if (isMoving == true && (abs(distanceToTarget) > distanceTolerance)) {

    if (abs(headingError) > angleTolerance) {
      velCommand.linear.x = 0.0;
      velCommand.angular.z = SF_A  * headingError;
    }
    else {
      velCommand.linear.x = SF_L * distanceToTarget;
      velCommand.angular.z = 0.0;
    }
  }
  else {
    cout << "Goal has been reached!" << endl << endl;
    velCommand.linear.x = 0.0;
    velCommand.angular.z = 0.0; 
    isMoving == false;
  }
}

void updatePose(const turtlesim::PoseConstPtr &currentPose) {
  currentPos.x = currentPose->x;
  currentPos.y = currentPose->y;
  currentPos.theta = currentPose->theta;
}

void updateTarget(const geometry_msgs::Pose2D &targetPose) {
  targetGoal.x = targetPose.x;
  targetGoal.y = targetPose.y;
  isMoving = true;
}

int main(int argc, char **argv) {

  setup();

  ros::init(argc, argv, "move_to_goal");

  ros::NodeHandle node;

  ros::Subscriber currentPoseSub =  node.subscribe("turtle1/pose", 0, updatePose);

//  ros::Subscriber waypointPoseSub =  node.subscribe("target", 0, updateTarget);
  ros::Subscriber waypointPoseSub =  node.subscribe("target", 0, updateTarget,  ros::TransportHints().tcpNoDelay());

  velocityPub =  node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    ros::spinOnce();

    setVelocity();

    velocityPub.publish(velCommand);

    cout << "Current Position (x,y) = " << "(" << currentPos.x << "," << currentPos.y << ")"
         << endl
         << "Target Goal (x,y) = " << "(" << targetGoal.x << ","
         << targetGoal.y << ")"
         << endl
         << "Distance to Target = " << getDistanceToTarget() << " m"
         << endl
         << "Linear Velocity (x) = " << velCommand.linear.x << " m/s"
         << endl << endl;

    loop_rate.sleep();
  }

  return 0;
}
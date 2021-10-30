//#include <cstdlib>
#include "ros/ros.h"
#include "cpp_pkg/ManualCommands.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include <sstream>
#include <iostream> 
#include <queue>
#include <math.h>

#include <std_msgs/Float32.h>
#include <cpp_pkg/move_to_goalAction.h>
#include <cpp_pkg/move_to_goalGoal.h>
#include <cpp_pkg/move_to_goalResult.h>
//#include <cpp_pkg/move_to_goalFeedbak.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<cpp_pkg::move_to_goalAction> Server;

using namespace std;


class Robot{
  private:
    geometry_msgs::Pose2D currentPos;// Current posiition
    geometry_msgs::Pose2D targetGoal; // Target GOAL (x, y)
    ros::Publisher velocityPub; // Object used for publishing velocity command
    ros::Publisher feedbackPub; // Object used for publishing feedback
    ros::ServiceServer service; 
    ros::ServiceClient client;


    ros::Subscriber currentPoseSub;
    ros::Subscriber targetPoseSub;

    cpp_pkg::move_to_goalFeedback feedback;

    const double PI = 3.141592654;
    const double SF_L = 1.5; //speed factor linear

    const double SF_A = 3.5; //speed factor angular

    const double distanceTolerance = 0.1;
    const double angleTolerance = 0.01; //0.01;
    bool isMoving = false;
    bool cleaner_mode = false;
    bool isFinish = false;
    

  protected:
    ros::NodeHandle node;
    actionlib::SimpleActionServer<cpp_pkg::move_to_goalAction> _as;
    

  public:
    geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 

    Robot() : _as(node, "/move_to_target", boost::bind(&Robot::cb, this, _1), false){
      
      _as.start();
      currentPoseSub =  node.subscribe("turtle1/pose", 0, &Robot::updatePose, this);

      //targetPoseSub =  node.subscribe("target", 0, &Robot::updateTarget, this);
      targetPoseSub =  node.subscribe("target", 0, &Robot::updateTarget, this, ros::TransportHints().tcpNoDelay());

      velocityPub =  node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

      feedbackPub = node.advertise<geometry_msgs::Pose2D>("feedback", 10);

      service = node.advertiseService("manual_commands", &Robot::process, this);

      //client = node.serviceClient<std_srvs::Empty>("/reset");
      client = node.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    }

    void setup() {
      targetGoal.x = 5.54;
      targetGoal.y = 5.54;
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
    // to turn to head towards the target
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

      if (isMoving == true){
        if(abs(distanceToTarget) > distanceTolerance) {
          if (cleaner_mode){
            if (abs(headingError) > angleTolerance) {
              velCommand.linear.x = 0.0;
              velCommand.angular.z = SF_A  * headingError;
            }
            else {
              velCommand.linear.x = SF_L * distanceToTarget;
              velCommand.angular.z = 0.0;
            }
          }
          else{
            velCommand.linear.x = SF_L * distanceToTarget;
            velCommand.linear.y = 0.0;
            velCommand.linear.z = 0.0;
            velCommand.angular.x = 0.0;
            velCommand.angular.y = 0.0;
            velCommand.angular.z = SF_A  * headingError;
          }
        }
        else {
          isFinish=true;
          ROS_INFO("Goal has been reached!");
          velCommand.linear.x = 0.0;
          velCommand.angular.z = 0.0; 
          isMoving = false;
          ros::spinOnce();
        }
      }
    }

    void updatePose(const turtlesim::PoseConstPtr &currentPose) {
      currentPos.x = currentPose->x;
      currentPos.y = currentPose->y;
      currentPos.theta = currentPose->theta;
      if(isMoving && !isFinish){
        //ROS_INFO("Sending feedback ...");
        feedbackPub.publish(currentPos); //feedbak topic
        feedback.x = currentPos.x;
        feedback.y = currentPos.y;
        ros::spinOnce();
      }
    }

    void updateTarget(const geometry_msgs::Pose2D &targetPose) {
      ROS_INFO("Goal received by topic");
      targetGoal.x = targetPose.x;
      targetGoal.y = targetPose.y;
      isMoving = true;
      isFinish = false;
      ROS_INFO("Target is : %f %f", targetPose.x, targetPose.y);
    }

    void processTarget(){
        if (isMoving){
          setVelocity();
	        velocityPub.publish(velCommand);
        }
    }


    void cb(const cpp_pkg::move_to_goalGoalConstPtr &goal){
      ROS_INFO("Goal received by action");
      geometry_msgs::Pose2D pose;
      pose.x=goal->x;
      pose.y=goal->y;
      
      ROS_INFO("Target is : %f %f", pose.x, pose.y);
      updateTarget(pose);

      ros::Rate rate(10);
      bool success = false;
      bool preempted = false;
      isFinish=false;
      while (ros::ok()){

        if (_as.isPreemptRequested()){
          preempted = true;
          break;
        }

        if (isFinish){
          success = true;
          break;
        }
        if(isMoving){
          _as.publishFeedback(feedback);
        }
        rate.sleep();
      }
      cpp_pkg::move_to_goalResult result;
      result.x = 0;
      result.y = 0;
      ROS_INFO("Send goal result to client");

      if (preempted){
        ROS_INFO("Preempted");
        _as.setPreempted(result);
      }
      else if (success){
        ROS_INFO("Success");
        _as.setSucceeded(result);
      }
      else{
        ROS_INFO("Aborted");
        _as.setAborted(result);
      }
    }
    bool process(cpp_pkg::ManualCommands::Request  &req, cpp_pkg::ManualCommands::Response &res) {

      if(req.order == "pause") {
          isMoving = false;
      }
      if(req.order == "resume") {
          isMoving = true;
      }
      if(req.order == "reset") {
          turtlesim::TeleportAbsolute absPos;
          absPos.request.x = 5.54;
          absPos.request.y = 5.54;
          absPos.request.theta = 0;
          isMoving = false;
          setup();
          client.call(absPos);
      }

        return true;
    }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "move_to_goal");

  Robot robot;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    robot.processTarget();
    loop_rate.sleep();
  }

  return 0;
}

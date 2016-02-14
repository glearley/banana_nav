//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley
// Version     : #2.0
// Copyright   : Your copyright notice
// Description : banana_nav_node
//==============================================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "banana_nav.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base/move_base.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <math.h>
#include <sstream>
#include <iostream>
using namespace std;


//typedef to simplify long object titles
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef nav_msgs::OccupancyGrid occupancyGrid;
typedef geometry_msgs::Pose pose;
occupancyGrid global_map;
void costmapcallback(occupancyGrid costmap){
	global_map = costmap;
}
int main(int argc, char** argv) {
	//First argument is field length
	//Second argument is field width
	ros::init(argc,argv,"banana_nav");
	//Used as a constructor and destructor of ROS nodes
	ros::NodeHandle banana_nav;
	//Object that allow banana_nav to publish
	ros::Publisher goal_pub = banana_nav.advertise<std_msgs::String>("goal", 1000);
	//Object that allow banana_nav to
	ros::Subscriber subOGrid = banana_nav.subscribe("move_base/local_costmap/costmap_update",1000,costmapcallback);
	//Rate that the goals are sent
	ros::Rate goal_rate(10);
	//custom message type to get goal from function
	Goal currentGoal;
	//message type to send goal to move_base
	move_base_msgs::MoveBaseGoal goal;
	//checks if there is an error in converting string to long
	char *p;
<<<<<<< HEAD
	//Set to true if we are at the end of the row and need to find new row
	bool endofRow = false;
	//Set to true if we have reached the end of the banana field
	bool done = false;
	//Holds the length and width of the local cost map
	int field_length = 0;
	int field_width = 0;
	//
	bool direction = true;

////////////////////////////////Take in inputs or go to defaults///////////////////////////////////////////////
	//if-else statements are there if no inputs are given and implements default values
	if(argc == 1)
=======
	long field_length;
	long field_width;
	if(argc == 2)
>>>>>>> d2117246b84681fc2c411cbc931b6d50ebb004e2
	{
		//use base_local param file if no inputs are given/////////////////////////////////////////////
		//banana_nav.getParam("field_length",field_length);
		//banana_nav.getParam("field_width",field_width);
		field_length = 6;
		field_width = 6;
	}
<<<<<<< HEAD
	else if(argc == 2){
		field_length = strtol(argv[1],&p,10);
		//banana_nav.getParam("field_width",field_width);
		field_width = 6;
=======
	else if(argc == 3){
	field_length = strtol(argv[2],&p,10);
	field_width = 5;
>>>>>>> d2117246b84681fc2c411cbc931b6d50ebb004e2
	}
	else if(argc == 4)
	{
<<<<<<< HEAD
		field_length = strtol(argv[1],&p,10);
		field_width = strtol(argv[2],&p,10);
	}	
	ROS_INFO("The field length is %ld",field_length);
	ROS_INFO("The field width is %ld",field_width);
=======
	field_length = strtol(argv[2],&p,10); 
	field_width = strtol(argv[3],&p,10);
	}	
	ROS_INFO("The field length is %ld ",field_length);
	ROS_INFO("The field width is %ld ",field_width);
>>>>>>> d2117246b84681fc2c411cbc931b6d50ebb004e2
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base",true);
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	//Move the base_link to the fond goal
	//initial check for trees
	while(~done){
		switch(endofRow){

		///////////////////////////////////On Row////////////////////////////////////////////////////////////////////

		//Base_link is currently on a row
		case false:
			endofRow = ~FindGoal(currentGoal,global_map,field_length,field_width);
			// check that base_link is the right frame
			goal.target_pose.header.frame_id = "base_link";
			goal.target_pose.header.stamp = ros::Time::now();
			//set the goals position
			goal.target_pose.pose.position.x = currentGoal.x;
			goal.target_pose.pose.position.y = currentGoal.y;
			goal.target_pose.pose.orientation.w = 1;
			ROS_INFO("Sending goal x = %f and y = %f",currentGoal.x,currentGoal.y);
			//send goal to move_base
			ac.sendGoal(goal);
			//Wait for goal to be completed
			ac.waitForResult();
			//Check if goal was successful. Break if it was not.
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Hooray, J5 moved to goal.");
			else
			{
				//goal failed -> end node
				ROS_INFO("The J5 didn't move to goal. Error...Error...Error");
				done = true;
				return 0;
			}
			endofRow = ~FindGoal(currentGoal,global_map,field_length,field_width);
			break;
			//////////////////////////////Find Next Row////////////////////////////////////////////////////////////////////////////////
			//Need to find next row
		case true:
			FindRow(currentGoal,global_map,field_length,field_width,direction);
			// check that base_link is the right frame
			goal.target_pose.header.frame_id = "base_link";
			goal.target_pose.header.stamp = ros::Time::now();

			//set the goals position
			goal.target_pose.pose.position.x = currentGoal.x;
			goal.target_pose.pose.position.y = currentGoal.y;
			//Turns base_link around to look for trees depending on direction
			if(currentGoal.orientation)
			//Need to check if this works*********************************************
			goal.target_pose.pose.orientation = tf::createQuaternionFromYaw(M_PI);
			else goal.target_pose.pose.orientation = tf::createQuaternionFromYaw(0);
			ROS_INFO("Sending goal x = %f and y = %f and orientation is %b",currentGoal.x,currentGoal.y,currentGoal.orientation);
			//send goal to move_base
			ac.sendGoal(goal);
			//Wait for goal to be completed
			ac.waitForResult();
			//Check if goal was successful.
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("Hooray, J5 moved to next row.");
				endofRow = false;
			}
			else{
				//goal failed -> end node
				ROS_INFO("The J5 didn't turn to next row. Error...Error...Error");
				done = true;
				return 0;
				break;
			}
			done = CheckifDone( global_map, field_length, field_width);
			break;
		default:
			//Should never get to default case
			ROS_INFO("ERROR ERROR ERROR in banana_nav_node");
			break;
		}
	}

<<<<<<< HEAD
	///////////////////////////////////Return Home/////////////////////////////////////////////////

	//Use map frame so you can return to home
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	//set the goals position
	goal.target_pose.pose.position.x = 0;
	goal.target_pose.pose.position.y = 0;
	goal.target_pose.pose.orientation = tf::createQuaternionFromYaw(0);
	ROS_INFO("Sending goal x = %d and y = %d and orientation is front",0,0);
	//send goal to move_base
=======
	move_base_msgs::MoveBaseGoal goal;
	//Simple test goal move robot 1/10 meter forward
// [ERROR]: The goal pose passed to this planner must be in the map frame.  It is instead in the j5 frame.
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = strtol(argv[1],&p,10);
	goal.target_pose.pose.position.y = 0;
	goal.target_pose.pose.position.z = 0;
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	if(strtol(argv[1],&p,10) < 0)	
	goal.target_pose.pose.orientation.w = 1;
	else goal.target_pose.pose.orientation.w = 1;

	ROS_INFO("Sending goal");
>>>>>>> d2117246b84681fc2c411cbc931b6d50ebb004e2
	ac.sendGoal(goal);
	//Wait for goal to be completed
	ac.waitForResult();
	//Check if goal was successful.
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Hooray, J5 Phoned Home and got there");
	}
	else{
		//goal failed -> end node
		ROS_INFO("The J5 didn't go home. Error...Error...Error");
	}
	return 0;
}


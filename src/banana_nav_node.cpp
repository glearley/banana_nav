//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley
// Version     : #1.0
// Copyright   : Your copyright notice
// Description : Hello World in C++,Ansi-style
//==============================================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sstream>
#include <iostream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	int main(int argc, char** argv) {
	//First argument is field length
	//Second argument is field width
	ros::init(argc,argv,"banana_nav");
	//Used to constructor and destructor nodes
	ros::NodeHandle banana_nav;
	//Object that allow banana_nav to publish
	ros::Publisher goal_pub = banana_nav.advertise<std_msgs::String>("goal", 1000);
	//Rate that the goals are sent
	ros::Rate goal_rate(10);
	char *p;
	long field_length;
	long field_width;
	if(argc == 1)
	{
	field_length = 5;
	field_width = 5;
	}
	else if(argc == 2){
	field_length = strtol(argv[1],&p,10);
	field_width = 5;
	}
	else if(argc == 3)
	{
	field_length = strtol(argv[1],&p,10); 
	field_width = strtol(argv[2],&p,10);
	}	
	ROS_INFO("The field length is %d ",field_length);
	ROS_INFO("The field width is %d ",field_width);
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base",true);
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
		{
		ROS_INFO("Waiting for the move_base action server to come up");
		}

	move_base_msgs::MoveBaseGoal goal;
	//Simple test goal move robot 1/10 meter forward
// [ERROR]: The goal pose passed to this planner must be in the map frame.  It is instead in the j5 frame.
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 0.1;
	goal.target_pose.pose.orientation.w = 1;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, J5 moved");
	else
		ROS_INFO("The J5 didn't move. Error...Error...Error");

	return 0;
}

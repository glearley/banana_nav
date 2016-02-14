//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley
// Version     : #1.0
// Copyright   : Your copyright notice
// Description : Hello World in C++,Ansi-style
//==============================================================================
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>
//#include <sstream>
//using namespace std;
//
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//
//	int main(int argc, char** argv) {
//	//First argument is field length
//	//Second argument is field width
//	ros::init(argc,argv,"banana_nav");
//	//Used to constructor and destructor nodes
//	ros::NodeHandle banana_nav;
//	//Object that allow banana_nav to publish
//	ros::Publisher goal_pub = banana_nav.advertise<std_msgs::String>("goal", 1000);
//	//Rate that the goals are sent
//	ros::Rate goal_rate(10);
//	ROS_INFO("The field length is %d ",argv[1]);
//	ROS_INFO("The field width is %d ",argv[2]);
//	//tell the action client that we want to spin a thread by default
//	MoveBaseClient ac("move_base",true);
//	//wait for the action server to come up
//	while(!ac.waitForServer(ros::Duration(5.0)))
//		{
//		ROS_INFO("Waiting for the move_base action server to come up");
//		}
//
//	move_base_msgs::MoveBaseGoal goal;
//	//Simple test goal move robot 1/10 meter forward
//	goal.target_pose.header.frame_id = "J5";
//	goal.target_pose.header.stamp = ros::Time::now();
//	goal.target_pose.pose.position.x = 0.1;
//	goal.target_pose.pose.orientation.w = 1;
//
//	ROS_INFO("Sending goal");
//	ac.sendGoal(goal);
//	ac.waitForResult();
//
//	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//		ROS_INFO("Hooray, J5 moved");
//	else
//		ROS_INFO("The J5 didn't move. Error...Error...Error");
//
//	return 0;
//}
#include "ros/ros.h"
#include <banana_nav/banana_nav.h>
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
typedef std::vector<int8_t> int8;


//include the costmap dependencies whenever I find that shiz
bool FindGoal(Goal currentGoal,int8 map, int m_x,int m_y){

	int maxHeight = m_y;
	int maxWidth = m_x;
	int height = 0;
	int width = 0;
	bool RTtrigger = false;
	bool LTtrigger = false;
	//Holds the cost of the tree on the left and right
	int RTcost = 0;
	int LTcost = 0;
	//variable that holds half the width of the map
	int halfWidth =maxWidth/2;
	//variables that hold the location of the closet tree on the left and right
	int LTLocatedHeight = 0;
	int LTLocatedWidth = 0;
	int RTLocatedHeight = 0;
	int RTLocatedWidth = 0;

	//while left tree and right tree have not been found
	for(height = maxHeight/2; height<maxHeight; height++){
		//search for the closet tree/obstacle to the left of the base_link
		for(width = halfWidth; width>=0;width--) {
			if(width<halfWidth){
				LTcost = GetCost(width,height,map, maxWidth, maxHeight);
//add in check if index is outside of map/////////////////////////////////////////////////////////////////////////////////////
				if ((LTcost == 100) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
					LTLocatedHeight = height;
					LTLocatedWidth = width;
				}
			}
		}
		//search for the closet tree/obstacle to the right of the base_link
		for(width = halfWidth; width < maxWidth;width++){
			//Find first tree on right side of base_link
			if(width>halfWidth){
				RTcost = GetCost(width,height,map, maxWidth, maxHeight);
				if ((RTcost == 100) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					RTtrigger = true;
					RTLocatedHeight = height;
					RTLocatedWidth = width;
				}
			}
		}
	}
	//Return false if no trees and set goal to null
	if((RTtrigger&&LTtrigger)!= true){//Check if there are no trees
		currentGoal.y = 0;
		currentGoal.x = 0;
		return false;
	}
	//Return true, we have found two trees and have a goal
	else if((RTtrigger&&LTtrigger) == true){
		//Sets the goal points based how the center of the local map which is the location of the base_link
		currentGoal.y = -1*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2);//negative because Y to the left is positive
		currentGoal.x = (RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2;
		return true;

		//unused path planing code may use in future
		/*	if (RTLocatedHeight == LTLocatedHeight){//accounts for the lethal points to be on same row
		goalHeight = RTLocatedHeight;
		goalWidth = (RTLocatedWidth+LTLocatedWidth)/2; //midpoint formula
	}
	else if (LTLocatedHeight<RTLocatedHeight) {//left tree closer than right tree
		goalHeight = LTLocatedHeight; //assume the closest tree to be the best stopping point
		goalWidth = (RTLocatedWidth+LTLocatedWidth)/2; //midpoint formula
	}
	else { //right tree closer
		goalHeight = RTLocatedHeight; //assume the closest tree to be the best stopping point
		goalWidth = (RTLocatedWidth+LTLocatedWidth)/2; //midpoint formula
	}*/
	}
}


//////////////////////////////////////Get Cost//////////////////////////////////////////////////////////////////////////

//function to get cost from occupancy grid given coordinates
int GetCost(int x,int y,int8 map, int max_x,int max_y)
{
	int index = GetIndex(x,y,max_x,max_y);
	//return -10 if index is outside of range
	if(index == -1) return -10;
	else return map[index];
}

///////////////////////////////////////////////////////Get Index//////////////////////////////////////////////////////////////////

//function to get index from occupancy grid given coordinates
int GetIndex(int x,int y, int max_x,int max_y)
{
	int index = 0;
	//return -1 if index is outside of range
	if((y > max_y) || (x > max_x)) return -1;
	//use number of rows minus 1 times the width of a row + the x coordinate to get the index
	else index = ((y-1)*max_x)+x;
	return index;
}


////////////////////////////////////////////////////////Find ROW/////////////////////////////////////////////////////////////////////////



//determines and gives the location of the next spot to look for trees
bool FindRow(Goal currentGoal,int8 map, int m_x,int m_y, bool direction)
{
	int maxHeight = m_y;
	int maxWidth = m_x;
	int height = 0;
	int width = 0;
	bool RTtrigger = false;
	bool LTtrigger = false;
	//Holds the cost of the tree on the left and right
	int RTcost = 0;
	int LTcost = 0;
	//variable that holds half the width of the map
	int halfWidth = maxWidth/2;
	//variable that holds the location of the closet tree on the left and right
	int LTLocatedHeight = 0;
	int LTLocatedWidth = 0;
	int RTLocatedHeight = 0;
	int RTLocatedWidth = 0;
	//amount to move in the x direction away from next row
	double heightOffset = 1.2;
	//amount to move in the y direction past the first tree on the next row
	double widthOffset = .5;

	//depending on the direction the goal will be on a different side
	switch(direction)
	{
	//enact if base_link is facing positive x
	case true:
		for(height = maxHeight; height>0; height--){
			//search for the closet tree/obstacle to the left of the base_link
			for(width = halfWidth; width<maxWidth;width++) {
				if(width<halfWidth){
					RTcost = GetCost(width,height,map, maxWidth, maxHeight);
					if ((RTcost == 100) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						RTtrigger = true;
						RTLocatedHeight = height;
						RTLocatedWidth = width;
					}
				}
			}
		}
		if(RTtrigger == false){
			//No tree found error
			currentGoal.x = 0;
			currentGoal.y = 0;
			return false;
		}
		else{
			//Move to in front of next row
			currentGoal.x = RTLocatedHeight + heightOffset;
			currentGoal.y = RTLocatedWidth + widthOffset;
			//Set goal so base_link is facing negative x
			currentGoal.orientation = false;
			return true;
		}
		break;
		//enact if base_link is facing negative x
	case false:
		for(height = maxHeight; height>0; height--){
			//search for the closet tree/obstacle to the left of the base_link
			for(width = halfWidth; width>=0;width--) {
				if(width<halfWidth){
					LTcost = GetCost(width,height,map, maxWidth, maxHeight);
					if ((LTcost == 100) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						LTtrigger = true;
						LTLocatedHeight = height;
						LTLocatedWidth = width;
					}
				}
			}
		}
		if(LTtrigger == false){
			//No tree found Error
			currentGoal.x = 0;
			currentGoal.y = 0;
			return false;
		}
		else{
			//Move to in front of next row
			currentGoal.x = RTLocatedHeight + heightOffset;
			currentGoal.y = RTLocatedWidth - widthOffset;
			//Set goal so base_link is facing positive x
			currentGoal.orientation = true;
			return true;
		}
		break;
	default:
			//Should never get to default case
			ROS_INFO("ERROR ERROR ERROR in FindRow");
			return false;
			break;
	}
}



//////////////////////////////////////////////////Check if Done/////////////////////////////////////////////////////////////////////

//determines if we are done and need to return to base
bool CheckifDone(int8 map, int m_x,int m_y){

	int maxHeight = m_y;
	int maxWidth = m_x;
	int height = 0;
	int width = 0;
	bool RTtrigger = false;
	bool LTtrigger = false;
	//Holds the cost of the tree on the left and right
	int RTcost = 0;
	int LTcost = 0;
	//variable that holds half the width of the map
	int halfWidth =maxWidth/2;


	for(height = maxHeight/2; height<maxHeight; height++){
			//search for the closet tree/obstacle to the left of the base_link
			for(width = halfWidth; width>=0;width--) {
				if(width<halfWidth){
					LTcost = GetCost(width,height,map, maxWidth, maxHeight);
					if ((LTcost == 100) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						LTtrigger = true;
					}
				}
			}
			//search for the closet tree/obstacle to the right of the base_link
			for(width = halfWidth; width < maxWidth;width++){
				//Find first tree on right side of base_link
				if(width>halfWidth){
					RTcost = GetCost(width,height,map, maxWidth, maxHeight);
					if ((RTcost == 100) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						RTtrigger = true;
					}
				}
			}
		}

	if(RTtrigger != true){//Check if there are no trees on right side
		return true;
	}
	else if(LTtrigger != true){//Check if there are no trees on left side
		return true;
	}
	else return false;
}


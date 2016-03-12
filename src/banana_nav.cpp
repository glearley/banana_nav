//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley and Justin Alabaster
// Version     : #3 with objects
// Copyright   : Your copyright notice
// Description : banana_nav library
//==============================================================================
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>

#include <banana_nav/banana_nav.h>

using namespace std;

typedef std::vector<int8_t> int8;

int buffer = 95;//Determines what cost values are considered objects

int frontFootPrintOffset = 60;//offset in cells
int sideFootPrintOffset = 60;//offset in cells


//Goal Constructor
Goal::Goal(double goal_x, double goal_y, bool goal_orientation)
	{
	this->x = goal_x;
	this->y = goal_y;
	this->orientation = goal_orientation;
	}

//Goal Destuctor
Goal::~Goal(){
}


//Banana_nav Constructor
Banana_nav::Banana_nav(Goal& init_goal,int8 init_map, int init_m_x, int init_m_y, float init_resolution){
this->currentGoal = &init_goal;
this->m_x = init_m_x;
this->m_y = init_m_y;
this->map = init_map;

//direction gives the direction base_link is facing.
//True means it is facing positive x and False means it is facing negative x
this->direction = true;

this->resolution = init_resolution;
}

Banana_nav::~Banana_nav(){
};


/////////////////////////////////////Private Methods//////////////////////////////////////////////////////////////////

//////////////////////////////////////Get Cost//////////////////////////////////////////////////////////////////////////

//function to get cost from occupancy grid given coordinates
int Banana_nav::GetCost(int width,int height)
{
	int maxHeight = this->m_x; int maxWidth = this->m_y;

	int index = this->GetIndex(width,height);

	//ROS_INFO("Value at index %d is %d", index, this.map[index]);//Used for Debug

	if(index == -1) return -10; //return -10 if index is outside of range
	else return this->map[index];
}

///////////////////////////////////////////////////////Get Index//////////////////////////////////////////////////////////////////

//function to get index from occupancy grid given coordinates
int Banana_nav::GetIndex(int width,int height)
{
	int index = 0;
	if((height > this->m_x) || (width > this->m_y)) return -1; //return -1 if index is outside of range
	else index = ((height)*this->m_y)+height; //use number of rows times the width of a row + the x coordinate to get the index
	return index;
}


///////////////////////////////////////////////////Public Methods///////////////////////////////////////////////////////////////


//Finds the goal and returns if it found one or not
bool Banana_nav::FindGoal(){

	//Stores the bounds of the map
	int maxHeight = this->m_x; int maxWidth = this->m_y;

	//for loop variables
	int height = 0; int width = 0;

	//Triggers that are set if a tree is found
	bool RTtrigger = false; bool LTtrigger = false;

	//Holds the cost of the tree on the left and right
	int RTcost = 0; int LTcost = 0;

	//variable that holds half the width of the map
	int halfWidth =maxWidth/2;

	//variables that hold the location of the closet tree on the left and right
	float LTLocatedHeight = 0; float LTLocatedWidth = 0;
	float RTLocatedHeight = 0; float RTLocatedWidth = 0;

	//while left tree and right tree have not been found
	for(height = maxHeight/2 + frontFootPrintOffset; height<=maxHeight; height++){

		for(width = halfWidth-sideFootPrintOffset; width>=0;width--) {//search for the closet tree/obstacle to the left of the base_link

				LTcost = this->GetCost(width,height);

				//ROS_INFO("LTcost = %d",LTcost);//Used for Debug

				//add in check if index is outside of map//////////////////////////////////////////////////////////////////////
				if ((LTcost > buffer) && (LTtrigger == false)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
					LTLocatedHeight = height;
					LTLocatedWidth = width;
				}

		}
		for(width = halfWidth+sideFootPrintOffset; width <= maxWidth;width++){//search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link
				RTcost = this->GetCost(width,height);

				//ROS_INFO("RTcost = %d",RTcost);//Used for Debug

				if ((RTcost > buffer) && (RTtrigger != true)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					RTtrigger = true;
					RTLocatedHeight = height;
					RTLocatedWidth = width;
				}
			}
		}
	}

	//Return false if no trees and sets goal to 0,0
	if(RTtrigger == false || LTtrigger == false){//Check if there are no trees

		ROS_INFO("I messed UP");//Displays if we don't see both trees
		if(LTtrigger)ROS_INFO("LT trigger is true");//Displays if we see tree on left side
		if(RTtrigger)ROS_INFO("RT trigger is true");//Displays if we see tree on right side

		this->currentGoal->y = 0;
		this->currentGoal->x = 0;
		return false;
	}

	//Return true, we have found two trees and have a goal
	else if((RTtrigger == true && LTtrigger == true)){

		//Location from bot in meters
		float MRTLocatedWidth = -this->resolution*(RTLocatedWidth-maxWidth/2);
		float MLTLocatedWidth = -this->resolution*(LTLocatedWidth-maxWidth/2);
		float MRTLocatedHeight = this->resolution*(RTLocatedHeight-maxHeight/2);
		float MLTLocatedHeight = this->resolution*(LTLocatedHeight-maxHeight/2);

		//Displays if we see trees
		ROS_INFO("I did the right thing \n right trigger is at height %f and width %f and left trigger is at height %f and width %f",MRTLocatedHeight,MRTLocatedWidth,MLTLocatedHeight,MLTLocatedWidth);

		//Sets the goal points based how the center of the local map which is the location of the base_link
		this->currentGoal->y = this->resolution*(-1*((RTLocatedWidth + LTLocatedWidth)/2 - (maxWidth/2)));//negative because Y to the left is positive
		this->currentGoal->x = this->resolution*((RTLocatedHeight +LTLocatedHeight)/2 - (maxHeight/2));

		//ROS_INFO("goal is at x = %f and y = %f",currentGoal.x,currentGoal.y); //Used for Debug
		return true;

	}
}

////////////////////////////////////////////////////////Find ROW/////////////////////////////////////////////////////////////////////////

//determines and gives the location of the next spot to look for trees
bool Banana_nav::FindRow()
{

	//Stores the bounds of the map
	int maxHeight = this->m_y; int maxWidth = this->m_x;

	//for loop variables
	int height = 0; int width = 0;

	//Triggers that are set if a tree is found
	bool RTtrigger = false; bool LTtrigger = false;

	//Holds the cost of the tree on the left and right
	int RTcost = 0; int LTcost = 0;

	//variable that holds half the width of the map
	int halfWidth =maxWidth/2;

	//variables that hold the location of the closet tree on the left and right
	int LTLocatedHeight = 0; int LTLocatedWidth = 0;
	int RTLocatedHeight = 0; int RTLocatedWidth = 0;

	double heightOffset = 1.2; //amount to move in the x direction away from next row

	double widthOffset = .5; //amount to move in the y direction past the first tree on the next row

	//depending on the direction the goal will be on a different side
	switch(direction)
	{

	//enact if base_link is facing positive x
	case true:
		for(height = maxHeight; height>0; height--){//Stop from top of
			//search for the closet tree/obstacle to the left of the base_link
			for(width = halfWidth+sideFootPrintOffset; width<=maxWidth;width++) {
				if(width<halfWidth){
					RTcost = this->GetCost(width,height);
					if ((RTcost > buffer) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						RTtrigger = true;
						RTLocatedHeight = height;
						RTLocatedWidth = width;
					}
				}
			}
		}
		if(RTtrigger == false){
			//No tree found error
			this->currentGoal->x = 0;
			this->currentGoal->y = 0;
			return false;
		}
		else{
			//Set goal to be in front of the next row
			this->currentGoal->y = this->resolution*(-1*(RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) - widthOffset;
			this->currentGoal->x = this->resolution*((RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2) + heightOffset;
			this->currentGoal->orientation = false;//Set goal so base_link is facing negative x
			return true;
		}
		break;
		//enact if base_link is facing negative x
	case false:
		for(height = maxHeight; height>0; height--){
			//search for the closet tree/obstacle to the left of the base_link
			for(width = halfWidth-sideFootPrintOffset; width>=0;width--) {
				if(width<halfWidth){
					LTcost = this->GetCost(width,height);
					if ((LTcost > buffer) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						LTtrigger = true;
						LTLocatedHeight = height;
						LTLocatedWidth = width;
					}
				}
			}
		}
		if(LTtrigger == false){
			//No tree found Error
			this->currentGoal->x = 0;
			this->currentGoal->y = 0;
			return false;
		}
		else{
			//Set goal to be in front of the next row
			this->currentGoal->y = -this->resolution*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) + widthOffset;
			this->currentGoal->x = this->resolution*(RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2 + heightOffset;
			this->currentGoal->orientation = true; //Set goal so base_link is facing positive x
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
bool Banana_nav::CheckifDone(){

	//Stores the bounds of the map
	int maxHeight = this->m_y; int maxWidth = this->m_x;

	//for loop variables
	int height = 0; int width = 0;

	//Triggers that are set if a tree is found
	bool RTtrigger = false; bool LTtrigger = false;

	//Holds the cost of the tree on the left and right
	int RTcost = 0; int LTcost = 0;

	//variable that holds half the width of the map
	int halfWidth = maxWidth/2;


	for(height = maxHeight/2+frontFootPrintOffset; height<=maxHeight; height++){

		for(width = halfWidth-sideFootPrintOffset; width>=0;width--) { //search for the closet tree/obstacle to the left of the base_link

			if(width<halfWidth){ //Find first tree on left side of base_link

				LTcost = this->GetCost(width,height);
				if ((LTcost > buffer) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
				}
			}
		}

		for(width = halfWidth+sideFootPrintOffset; width < maxWidth;width++){ //search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link

				RTcost = this->GetCost(width,height);
				if ((RTcost > buffer) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					RTtrigger = true;
				}
			}
		}
	}

	if(RTtrigger == true && LTtrigger == true){//Check if there are trees on both sides
		return false;
	}
	else return true;//There are no trees on either side
}



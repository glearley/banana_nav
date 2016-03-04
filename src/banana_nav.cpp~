//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley and Justin Alabaster
// Version     : #2.3 with improved formating and commenting
// Copyright   : Your copyright notice
// Description : banana_nav library
//==============================================================================
#include "ros/ros.h"
#include <banana_nav/banana_nav.h>
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

using namespace std;

typedef std::vector<int8_t> int8;

int buffer = 95;//Determines what cost values are considered objects

int footprintoffset = 80;//offset in cells

bool FindGoal(Goal &currentGoal,int8 map, int m_x,int m_y,float resolution,float rotation){

	float upperRange = M_PI/16 ;
	float lowerRange = 15*M_PI/16;

	//if(rotation > -upperRange && rotation < upperRange)
	return FindGoalFront(currentGoal,map,m_x,m_y,resolution);

	//else if(rotation < -lowerRange && rotation > lowerRange)
	//return FindGoalBack(currentGoal,map,m_x,m_y,resolution);

	//else
	ROS_INFO("Error Error J5 not directly facing positive x or negative x its relative rotation is %f",rotation);
	return false;
}


//Finds the goal and returns if it found one or not
bool FindGoalFront(Goal &currentGoal,int8 map, int m_x,int m_y,float resolution){

	//Stores the bounds of the map
	int maxHeight = m_y; int maxWidth = m_x;

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
	for(height = maxHeight/2 + footprintoffset; height<=maxHeight; height++){

		for(width = halfWidth; width>=0;width--) {//search for the closet tree/obstacle to the left of the base_link

				LTcost = GetCost(width,height,map, maxWidth, maxHeight);

				//ROS_INFO("LTcost = %d",LTcost);//Used for Debug

				//add in check if index is outside of map//////////////////////////////////////////////////////////////////////
				if ((LTcost > buffer) && (LTtrigger == false)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
					LTLocatedHeight = height;
					LTLocatedWidth = width;
				}

		}
		for(width = halfWidth; width <= maxWidth;width++){//search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link
				RTcost = GetCost(width,height,map, maxWidth, maxHeight);

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

		currentGoal.y = 0;
		currentGoal.x = 0;
		return false;
	}

	//Return true, we have found two trees and have a goal
	else if((RTtrigger == true && LTtrigger == true)){

		//Location from bot in meters
		float MRTLocatedWidth = -resolution*(RTLocatedWidth-maxWidth/2);
		float MLTLocatedWidth = -resolution*(LTLocatedWidth-maxWidth/2);
		float MRTLocatedHeight = resolution*(RTLocatedHeight-maxHeight/2);
		float MLTLocatedHeight = resolution*(LTLocatedHeight-maxHeight/2);

		//Displays if we see trees
		ROS_INFO("I did the right thing \n right trigger is at height %f and width %f and left trigger is at height %f and width %f",MRTLocatedHeight,MRTLocatedWidth,MLTLocatedHeight,MLTLocatedWidth);

		//Sets the goal points based how the center of the local map which is the location of the base_link
		currentGoal.y = resolution*(-1*((RTLocatedWidth + LTLocatedWidth)/2 - (maxWidth/2)));//negative because Y to the left is positive
		currentGoal.x = resolution*((RTLocatedHeight +LTLocatedHeight)/2 - (maxHeight/2));

		//ROS_INFO("goal is at x = %f and y = %f",currentGoal.x,currentGoal.y); //Used for Debug
		return true;

	}
}


//Finds the goal and returns if it found one or not
bool FindGoalBack(Goal &currentGoal,int8 map, int m_x,int m_y,float resolution){

	//Stores the bounds of the map
	int maxHeight = m_y; int maxWidth = m_x;

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
	for(height = maxHeight/2-offset; height>=0; height--){

		for(width = halfWidth; width>=0;width--) {//search for the closet tree/obstacle to the left of the base_link

				LTcost = GetCost(width,height,map, maxWidth, maxHeight);

				//ROS_INFO("LTcost = %d",LTcost);//Used for Debug

				//add in check if index is outside of map//////////////////////////////////////////////////////////////////////
				if ((LTcost > buffer) && (LTtrigger == false)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
					LTLocatedHeight = height;
					LTLocatedWidth = width;
				}

		}
		for(width = halfWidth; width <= maxWidth;width++){//search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link
				RTcost = GetCost(width,height,map, maxWidth, maxHeight);

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

		currentGoal.y = 0;
		currentGoal.x = 0;
		return false;
	}

	//Return true, we have found two trees and have a goal
	else if((RTtrigger == true && LTtrigger == true)){

		//Location from bot in meters
		float MRTLocatedWidth = -resolution*(RTLocatedWidth-maxWidth/2);
		float MLTLocatedWidth = -resolution*(LTLocatedWidth-maxWidth/2);
		float MRTLocatedHeight = resolution*(RTLocatedHeight-maxHeight/2);
		float MLTLocatedHeight = resolution*(LTLocatedHeight-maxHeight/2);

		//Displays if we see trees
		ROS_INFO("I did the right thing \n right trigger is at height %f and width %f and left trigger is at height %f and width %f",MRTLocatedHeight,MRTLocatedWidth,MLTLocatedHeight,MLTLocatedWidth);

		//Sets the goal points based how the center of the local map which is the location of the base_link
		currentGoal.y = resolution*(-1*((RTLocatedWidth + LTLocatedWidth)/2 - (maxWidth/2)));//negative because Y to the left is positive
		currentGoal.x = resolution*((RTLocatedHeight +LTLocatedHeight)/2 - (maxHeight/2));

		//ROS_INFO("goal is at x = %f and y = %f",currentGoal.x,currentGoal.y); //Used for Debug
		return true;

	}
}




//////////////////////////////////////Get Cost//////////////////////////////////////////////////////////////////////////

//function to get cost from occupancy grid given coordinates
int GetCost(int width,int height,int8 map, int max_width,int max_height)
{
	int index = GetIndex(width,height,max_width,max_height);

	//ROS_INFO("Value at index %d is %d", index, map[index]);//Used for Debug

	if(index == -1) return -10; //return -10 if index is outside of range
	else return map[index];
}

///////////////////////////////////////////////////////Get Index//////////////////////////////////////////////////////////////////

//function to get index from occupancy grid given coordinates
int GetIndex(int width,int height, int max_width,int max_height)
{
	int index = 0;
	if((height > max_height) || (width > max_width)) return -1; //return -1 if index is outside of range
	else index = ((height)*max_width)+height; //use number of rows times the width of a row + the x coordinate to get the index
	return index;
}


////////////////////////////////////////////////////////Find ROW/////////////////////////////////////////////////////////////////////////


bool FindRow(Goal& currentGoal,int8 map, int m_x,int m_y, bool direction, float resolution, float rotation){
	float upperRange = M_PI/16 ;
	float lowerRange = 15*M_PI/16;

//	if(rotation > -upperRange && rotation < upperRange)
	return FindRowFront(currentGoal,map,m_x,m_y,direction,resolution);

//	else if(rotation < -lowerRange && rotation > lowerRange)
//	return FindRowBack(currentGoal,map,m_x,m_y,direction,resolution);

//	else ROS_INFO("Error Error J5 not directly facing positive x or negative x its relative rotation is %f",rotation);
	return false;
}

//determines and gives the location of the next spot to look for trees
bool FindRowFront(Goal& currentGoal,int8 map, int m_x,int m_y, bool direction, float resolution)
{
	//Stores the bounds of the map
	int maxHeight = m_y; int maxWidth = m_x;

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
			for(width = halfWidth; width<=maxWidth;width++) {
				if(width<halfWidth){
					RTcost = GetCost(width,height,map, maxWidth, maxHeight);
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
			currentGoal.x = 0;
			currentGoal.y = 0;
			return false;
		}
		else{
			//Set goal to be in front of the next row
			currentGoal.y = resolution*(-1*(RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) - widthOffset;
			currentGoal.x = resolution*((RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2) + heightOffset;
			currentGoal.orientation = false;//Set goal so base_link is facing negative x
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
			currentGoal.x = 0;
			currentGoal.y = 0;
			return false;
		}
		else{
			//Set goal to be in front of the next row
			currentGoal.y = -resolution*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) + widthOffset;
			currentGoal.x = resolution*(RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2 + heightOffset;
			currentGoal.orientation = true; //Set goal so base_link is facing positive x
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

//determines and gives the location of the next spot to look for trees
bool FindRowBack(Goal& currentGoal,int8 map, int m_x,int m_y, bool direction, float resolution)
{
	//Stores the bounds of the map
	int maxHeight = m_y; int maxWidth = m_x;

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
		for(height = 0; height<=maxHeight; height++){//Stop from top of
			//search for the closet tree/obstacle to the left of the base_link
			for(width = halfWidth; width<maxWidth;width++) {
				if(width<halfWidth){
					RTcost = GetCost(width,height,map, maxWidth, maxHeight);
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
			currentGoal.x = 0;
			currentGoal.y = 0;
			return false;
		}
		else{
			//Set goal to be in front of the next row
			currentGoal.y = resolution*(-1*(RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) - widthOffset;
			currentGoal.x = resolution*((RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2) + heightOffset;
			currentGoal.orientation = false;//Set goal so base_link is facing negative x
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
			currentGoal.x = 0;
			currentGoal.y = 0;
			return false;
		}
		else{
			//Set goal to be in front of the next row
			currentGoal.y = -resolution*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) + widthOffset;
			currentGoal.x = resolution*(RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2 + heightOffset;
			currentGoal.orientation = true; //Set goal so base_link is facing positive x
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


bool CheckifDone(int8 map, int m_x,int m_y,float rotation){
	float upperRange = M_PI/16 ;
	float lowerRange = 15*M_PI/16;

//	if(rotation > -upperRange && rotation < upperRange)
	return CheckifDoneFront(map,m_x,m_y);

//	else if(rotation < -lowerRange && rotation > lowerRange)
//	return CheckifDoneBack(map,m_x,m_y);

//	else ROS_INFO("Error Error J5 not directly facing positive x or negative x its relative rotation is %f",rotation);
	return false;
}


//determines if we are done and need to return to base
bool CheckifDoneFront(int8 map, int m_x,int m_y){


	//Stores the bounds of the map
	int maxHeight = m_y; int maxWidth = m_x;

	//for loop variables
	int height = 0; int width = 0;

	//Triggers that are set if a tree is found
	bool RTtrigger = false; bool LTtrigger = false;

	//Holds the cost of the tree on the left and right
	int RTcost = 0; int LTcost = 0;

	//variable that holds half the width of the map
	int halfWidth = maxWidth/2;


	for(height = maxHeight/2; height<=maxHeight; height++){

		for(width = halfWidth; width>=0;width--) { //search for the closet tree/obstacle to the left of the base_link

			if(width<halfWidth){ //Find first tree on left side of base_link

				LTcost = GetCost(width,height,map, maxWidth, maxHeight);
				if ((LTcost > buffer) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
				}
			}
		}

		for(width = halfWidth; width < maxWidth;width++){ //search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link

				RTcost = GetCost(width,height,map, maxWidth, maxHeight);
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


bool CheckifDoneBack(int8 map, int m_x,int m_y){


	//Stores the bounds of the map
	int maxHeight = m_y; int maxWidth = m_x;

	//for loop variables
	int height = 0; int width = 0;

	//Triggers that are set if a tree is found
	bool RTtrigger = false; bool LTtrigger = false;

	//Holds the cost of the tree on the left and right
	int RTcost = 0; int LTcost = 0;

	//variable that holds half the width of the map
	int halfWidth = maxWidth/2;


	for(height = maxHeight/2; height>=0; height--){

		for(width = halfWidth; width>=0;width--) { //search for the closet tree/obstacle to the left of the base_link

			if(width<halfWidth){ //Find first tree on left side of base_link

				LTcost = GetCost(width,height,map, maxWidth, maxHeight);
				if ((LTcost > buffer) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
				}
			}
		}

		for(width = halfWidth; width < maxWidth;width++){ //search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link

				RTcost = GetCost(width,height,map, maxWidth, maxHeight);
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



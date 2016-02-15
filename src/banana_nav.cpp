//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley
// Version     : #2.1 with improved formating and commenting
// Copyright   : Your copyright notice
// Description : banana_nav library
//==============================================================================
#include "ros/ros.h"
#include <banana_nav/banana_nav.h>
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
typedef std::vector<int8_t> int8;


//Finds the goal and returns if it found one or not
bool FindGoal(Goal currentGoal,int8 map, int m_x,int m_y){

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

	//while left tree and right tree have not been found
	for(height = maxHeight/2; height<maxHeight; height++){

		for(width = halfWidth; width>=0;width--) {//search for the closet tree/obstacle to the left of the base_link

			if(width<halfWidth){
				LTcost = GetCost(width,height,map, maxWidth, maxHeight);

				//add in check if index is outside of map//////////////////////////////////////////////////////////////////////
				if ((LTcost == 100) && (LTtrigger != true)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
					LTLocatedHeight = height;
					LTLocatedWidth = width;
				}
			}
		}
		for(width = halfWidth; width < maxWidth;width++){//search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link
				RTcost = GetCost(width,height,map, maxWidth, maxHeight);

				if ((RTcost == 100) && (RTtrigger != true)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					RTtrigger = true;
					RTLocatedHeight = height;
					RTLocatedWidth = width;
				}
			}
		}
	}

	//Return false if no trees and sets goal to 0,0
	if((RTtrigger&&LTtrigger)!= true){//Check if there are no trees
		currentGoal.y = 0;
		currentGoal.x = 0;
		return false;
	}

	//Return true, we have found two trees and have a goal
	else if((RTtrigger&&LTtrigger) == true){

		//Sets the goal points based how the center of the local map which is the location of the base_link
		currentGoal.y = -1*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) + maxWidth/2;//negative because Y to the left is positive
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
	if(index == -1) return -10; //return -10 if index is outside of range
	else return map[index];
}

///////////////////////////////////////////////////////Get Index//////////////////////////////////////////////////////////////////

//function to get index from occupancy grid given coordinates
int GetIndex(int x,int y, int max_x,int max_y)
{
	int index = 0;
	if((y > max_y) || (x > max_x)) return -1; //return -1 if index is outside of range
	else index = ((y-1)*max_x)+x; //use number of rows minus 1 times the width of a row + the x coordinate to get the index
	return index;
}


////////////////////////////////////////////////////////Find ROW/////////////////////////////////////////////////////////////////////////



//determines and gives the location of the next spot to look for trees
bool FindRow(Goal currentGoal,int8 map, int m_x,int m_y, bool direction)
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
			//Set goal to be in front of the next row
			currentGoal.y = -1*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) - widthOffset;
			currentGoal.x = (RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2 + heightOffset;
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
			//Set goal to be in front of the next row
			currentGoal.y = -1*((RTLocatedWidth + LTLocatedWidth)/2 - maxWidth/2) + widthOffset;
			currentGoal.x = (RTLocatedHeight +LTLocatedHeight)/2 - maxHeight/2 + heightOffset;
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

//determines if we are done and need to return to base
bool CheckifDone(int8 map, int m_x,int m_y){


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


	for(height = maxHeight/2; height<maxHeight; height++){

		for(width = halfWidth; width>=0;width--) { //search for the closet tree/obstacle to the left of the base_link

			if(width<halfWidth){ //Find first tree on left side of base_link

				LTcost = GetCost(width,height,map, maxWidth, maxHeight);
				if ((LTcost == 100) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
				}
			}
		}

		for(width = halfWidth; width < maxWidth;width++){ //search for the closet tree/obstacle to the right of the base_link

			if(width>halfWidth){ //Find first tree on right side of base_link

				RTcost = GetCost(width,height,map, maxWidth, maxHeight);
				if ((RTcost == 100) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					RTtrigger = true;
				}
			}
		}
	}

	if(RTtrigger == true && LTtrigger == true){//Check if there are trees on both sides
		return true;
	}
	else return false;//There are no trees on one side
}


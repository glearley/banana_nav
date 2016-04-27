//============================================================================
// Name        : banana_nav.cpp
// Author      : Gabriel Earley and Justin Alabaster
// Version     : #4 with objects
// Copyright   : Your copyright notice
// Description : banana_nav library
//==============================================================================
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

#include <banana_nav/banana_nav.h>

using namespace std;

typedef std::vector<int8_t> int8;

int buffer = 95;//Determines what cost values are considered objects

int cellsInTree = 0;//Number of obstacle points in a row that categorizes object as tree

int frontFootPrintOffset = 10;//front offset in cells
int sideFootPrintOffset = 10;//interiar side offset in cells
float robotFootPrint = .765;//Robot foot print in meters


/////////////////////////Class Constructors and Destructors/////////////////////////////////////

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
Banana_nav::Banana_nav(Goal& init_goal,int8 init_map, int init_m_x, int init_m_y, float init_resolution)
{	
	this->currentGoal = &init_goal;
	this->m_x = init_m_x;
	this->m_y = init_m_y;
	this->map = init_map;

	//direction gives the direction base_link is facing.	
	this->direction = true; //True means it is facing positive x and False means it is facing negative x

	this->resolution = init_resolution;
}

//Banana_nav Destuctor
Banana_nav::~Banana_nav(){
};


/////////////////////////////////////Private Methods//////////////////////////////////////////////////////////////////

//////////////////////////////////////Get Cost//////////////////////////////////////////////////////////////////////////

//function to get cost from occupancy grid given coordinates
int Banana_nav::GetCost(int width,int height)
{
	int index = this->GetIndex(width,height);
	//ROS_INFO("Value at index %d is %d", index, this->map[index]);//Used for Debug
	if(index == -1) return -10; //return -10 if index is outside of range
	else return this->map[index];
}

///////////////////////////////////////////////////////Get Index//////////////////////////////////////////////////////////////////

//function to get index from occupancy grid given coordinates
int Banana_nav::GetIndex(int width,int height)
{
	int index = 0;
	if((height >= this->m_x) || (width >= this->m_y))
	{
 	ROS_INFO("Error location is outside of index");
	return -1; //return -1 if index is outside of range
	}	
	else index = ((height)*this->m_y)+width; //use number of rows times the width of a row + the x coordinate to get the index
	return index;
}

////////////////////////////////////////////////////Tree Check/////////////////////////////////////////////////////////////////////

bool Banana_nav::TreeCheckRight(int width,int height){
	if((width + cellsInTree) > m_x){
		ROS_INFO("Error: Potental Tree Outside of Map to the right");	
		return false;
	}
	for(int i = 0;i <= cellsInTree;i++){
		if (buffer > this->GetCost(width+i,height)) return false;	
		}
	return true;
}

bool Banana_nav::TreeCheckLeft(int width,int height){
	if((width - cellsInTree) < 0){
		ROS_INFO("Error: Potental Tree Outside of Map to the left");	
		return false;
	}
	for(int i = 0;i <= cellsInTree;i++){
		if (buffer > this->GetCost(width-i,height)) return false;	
		}
	return true;
}



///////////////////////////////////////////////////Public Methods///////////////////////////////////////////////////////////////


/////////////////////////////////////////////Find Goal//////////////////////////////////////////////////////////

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
	for(width = maxWidth/2 + frontFootPrintOffset; width<maxHeight; width++){

		for(height = maxHeight/2+sideFootPrintOffset; height<maxHeight;height++) {//search for the closet tree/obstacle to the left of the base_link

				LTcost = this->GetCost(width,height);

				//ROS_INFO("LTcost = %d",LTcost);//Used for Debug

//////////////////////////////////////////////add in check if index is outside of map//////////////////////////////////////
				if ((LTcost > buffer) && (LTtrigger == false)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					if(this->TreeCheckLeft(width,height))
						{
						LTtrigger = true;
						LTLocatedHeight = height;
						LTLocatedWidth = width;
						}
				}

		}
		for(height = maxHeight/2-sideFootPrintOffset; height >= 0;height--){//search for the closet tree/obstacle to the right of the base_link
				RTcost = this->GetCost(width,height);

				//ROS_INFO("RTcost = %d",RTcost);//Used for Debug

				if ((RTcost > buffer) && (RTtrigger != true)){//found a lethal cost and haven't found one before this.									//lethal points are trees and it WILL NOT see last layers trees
					if(this->TreeCheckRight(width,height)){					
						RTtrigger = true;
						RTLocatedHeight = height;
						RTLocatedWidth = width;
					}				
			}
		}
	}

	//Return false if no trees and sets goal to 0,0
	if(RTtrigger == false && LTtrigger == false){//Checks if there are no trees

		ROS_INFO("I see no trees");//Displays if we don't see both trees
		this->currentGoal->y = 0;
		this->currentGoal->x = 0;
		return false;
	}

	//Return true if their is 1 tree to the left and sets goal to be at its height and its width is a distance of 1.5 times the robots footprint
	if(RTtrigger == false && LTtrigger == true){//Checks if there is one tree to the left

		ROS_INFO("I see 1 tree to my left");//Displays conformation message
		this->currentGoal->x = this->resolution*(LTLocatedWidth-maxWidth/2);// + 1.5*robotFootPrint;
		this->currentGoal->y = this->resolution*(LTLocatedHeight-maxHeight/2) - 1.5*robotFootPrint;
		return true;
	}

	//Return true if their is 1 tree to the right and sets goal to be at its height and its width is a distance of 1.5 times the robots footprint
	if(RTtrigger == true && LTtrigger == false){//Checks if there is one tree to the right

		ROS_INFO("I see 1 tree to my right");//Displays conformation message

		this->currentGoal->x = this->resolution*(RTLocatedWidth-maxWidth/2);// - 1.5*robotFootPrint;
		this->currentGoal->y = this->resolution*(RTLocatedHeight-maxHeight/2) + 1.5*robotFootPrint;
		return true;
	}

	//Return true, we have found two trees and have a goal
	else if((RTtrigger == true && LTtrigger == true)){

		//Location from bot in meters for Debug
		float MRTLocatedWidth = -this->resolution*(RTLocatedWidth-maxWidth/2);
		float MLTLocatedWidth = -this->resolution*(LTLocatedWidth-maxWidth/2);
		float MRTLocatedHeight = this->resolution*(RTLocatedHeight-maxHeight/2);
		float MLTLocatedHeight = this->resolution*(LTLocatedHeight-maxHeight/2);

		//Displays if we see trees
		ROS_INFO("I see two trees \n right trigger is at height %f and width %f and left trigger is at height %f and width %f",MRTLocatedWidth,-MRTLocatedHeight,MLTLocatedWidth,-MLTLocatedHeight);

		//Sets the goal points based how the center of the local map which is the location of the base_link
		this->currentGoal->x = this->resolution*(1*((RTLocatedWidth + LTLocatedWidth)/2 - (maxWidth/2)));//negative because Y to the left is positive
		this->currentGoal->y = this->resolution*((RTLocatedHeight +LTLocatedHeight)/2 - (maxHeight/2));

		//ROS_INFO("goal is at x = %f and y = %f",this->currentGoal->x,this->currentGoal->y); //Used for Debug
		return true;
	}
}

////////////////////////////////////////////////////////Find ROW/////////////////////////////////////////////////////////////////////////

//determines and gives the location of the next spot to look for trees
bool Banana_nav::FindRow()
{

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
	int LTLocatedHeight = 0; int LTLocatedWidth = 0;
	int RTLocatedHeight = 0; int RTLocatedWidth = 0;

	double heightOffset = 1.2; //amount to move in the x direction away from next row

	double widthOffset = 1.5; //amount to move in the y direction past the first tree on the next row

	//depending on the direction the goal will be on a different side
	switch(direction)
	{

	//enact if base_link is facing positive x
	case true:
		for(width = maxWidth-1; width>=0; width--){//Stop from top of
			//search for the closet tree/obstacle to the left of the base_link
			for(height = maxHeight/2; height>=0;height--) {
					RTcost = this->GetCost(width,height);
					if ((RTcost > buffer) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						RTtrigger = true;
						RTLocatedHeight = height;
						RTLocatedWidth = width;
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
			this->currentGoal->x = this->resolution*(RTLocatedWidth - maxWidth/2) + heightOffset;
			this->currentGoal->y = this->resolution*(RTLocatedHeight - maxHeight/2) - widthOffset;
			this->currentGoal->orientation = false;//Set goal so base_link is facing negative x
			return true;
		}
		break;
		//enact if base_link is facing negative x
	case false:
		for(width = maxWidth-1; width>=0; width--){
			//search for the closet tree/obstacle to the left of the base_link
			for(height = maxHeight/2; height < maxHeight;height++) {
					LTcost = this->GetCost(width,height);
					if ((LTcost > buffer) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
						LTtrigger = true;
						LTLocatedHeight = height;
						LTLocatedWidth = width;
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
			this->currentGoal->x = this->resolution*(LTLocatedWidth - maxWidth/2) + heightOffset;
			this->currentGoal->y = this->resolution*(LTLocatedHeight - maxHeight/2) + widthOffset;
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

	for(height = maxHeight/2+frontFootPrintOffset; height<maxHeight; height++){

		for(width = halfWidth-sideFootPrintOffset; width>=0;width--) { //search for the closet tree/obstacle to the left of the base_link


				LTcost = this->GetCost(width,height);
				if ((LTcost > buffer) && (LTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					LTtrigger = true;
				}
			}

		for(width = halfWidth+sideFootPrintOffset; width < maxWidth;width++){ //search for the closet tree/obstacle to the right of the base_link

				RTcost = this->GetCost(width,height);
				if ((RTcost > buffer) && (RTtrigger != true)){//found a lethal cost and haven't found one before this. Assumes only 										//lethal points are trees and it WILL NOT see last layers trees
					RTtrigger = true;
				}
			}
		}

	if(RTtrigger == true || LTtrigger == true){//Check if there are trees on either both sides
		return false;
	}
	else return true;//There are no trees on either side
}

//From the code of DLu
void Banana_nav::UpdateMap(const map_msgs::OccupancyGridUpdate& msg)
{
    int index = 0;
    for(int y=msg.y; y< msg.y+msg.height; y++){
    	for(int x=msg.x; x< msg.x+msg.width; x++){
    		this->map[this->GetIndex(x,y)] = msg.data[index++]; 
    	}
    }
}








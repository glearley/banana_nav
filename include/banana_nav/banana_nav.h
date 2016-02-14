/*
 * banana_nav.h

 *
 *  Created on: Jan 31, 2016
 *      Author: Gabriel Earley
 */

#ifndef BANANA_NAV_H_
#define BANANA_NAV_H_
typedef std::vector<int8_t> int8;
//custom class to hold goal before sending it to move_base
//Members: x,y, orientation give location of goal relative to base_link
//done tells if path planning is over and if it needs to head back to home
class Goal{
public:
	//look in to adding default values here
	double x;
	double y;
	bool orientation;// true is face positive x; false is face negative x
	Goal(double a, double b, bool c)
	{
	x = a;
	y = b;
	orientation =c;
	};
	~Goal(){}
};

//function to get cost from occupancy grid given coordinates
//Param: x,y are location of point. map is occupancyGrid you are wanting information from. max_x, max_y are constraints of map.
int GetCost(int x,int y,int8 map, int max_x,int max_y);


//function to get index from occupancy grid given coordinates
//Param: x,y are location of point. max_x, max_y are constraints of map.
int GetIndex(int x,int y,int max_x,int max_y);


//Finds the goal and returns if it found one or not
//Param: currentGoal is object you store goal coordinates in. map is occupancyGrid you are wanting information from. max_x, max_y are constraints of map.
bool FindGoal(Goal currentGoal,int8 map,int m_x,int m_y);


//determines and gives the location of the next spot to look for trees
//Param: x,y are location of point. map is occupancyGrid you are wanting information from. max_x, max_y are constraints of map.
//direction is the direction the base_link is facing with respect to the world positive x is true and negative x is false
bool FindRow(Goal currentGoal,int8 map, int m_x,int m_y, bool direction);


//determines if we are done and need to return to base
////Param: x,y are location of point. map is occupancyGrid you are wanting information from. max_x, max_y are constraints of map.
//
bool CheckifDone(int8 map, int m_x,int m_y);



#endif /* BANANA_NAV_H_ */



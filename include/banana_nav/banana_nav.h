/*
 * banana_nav.h

 *
 *  Created on: 2/14/2016
 *      Version 3 with objects
 *      Author: Gabriel Earley and Justin Alabaster
 */

#ifndef BANANA_NAV_H_
#define BANANA_NAV_H_

typedef std::vector<int8_t> int8;

//custom class to hold goal before sending it to move_base
//Members: x,y, orientation give location of goal relative to base_link
//Members: done tells if path planning is over and if it needs to head back to home
class Goal{
public:
	//location of goal
	double x; double y;

	bool orientation;// true is face positive x; false is face negative x

	Goal(double goal_x, double goal_y, bool goal_orientation);// Constructor
	~Goal();//Destructor
};




//Param: map is the array you are wanting information from. max_x, max_y are bounds of map.
//Param: direction is the direction the base_link is facing with respect to the world positive x is true and negative x is false
//Param: resolution is the number of meters per cell in the map. rotation is how far the vehicle has rotated off its initial axis
class Banana_nav{
public:
Goal* currentGoal;
int m_x,m_y;
int8 map;
bool direction;
float resolution;

Banana_nav(Goal& init_goal,int8 init_map, int init_m_x, int init_m_y, float init_resolution);

~Banana_nav();

//Finds the goal and returns if it found one or not
//Param: currentGoal is object you store goal coordinates in. map is the array you are wanting information from.
//Param: max_x, max_y are bounds of map. resolution is the number of meters per cell in the map.
//Param: rotation is how far the vehicle has rotated off its initial axis
bool FindGoal();

//determines and gives the location of the next spot to look for trees
bool FindRow();

//determines if we are done and need to return to base
//Param: x,y are location of point. map is the array you are wanting information from.
//Param: max_x, max_y are bounds of map. rotation is how far the vehicle has rotated off its initial axis
bool CheckifDone();

private:
//function to get cost from occupancy grid given coordinates
//Param: width ,height are location of point
int GetCost(int width, int height);

//function to get index from occupancy grid given coordinates
//Param: width ,height are location of point
int GetIndex(int width, int height);

};


#endif /* BANANA_NAV_H_ */



/*
 * banana_nav.h

 *
 *  Created on: 3/12/2016
 *      Version 3.01 with objects
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
	double x; double y;//location of goal

	bool orientation;// true is face positive x; false is face negative x

	Goal(double goal_x, double goal_y, bool goal_orientation);// Constructor
	~Goal();//Destructor
};

class Banana_nav{
public:
	Goal* currentGoal;//currentGoal is the object the goal's coordinates are stored in
	
	
	int m_x,m_y;//max_x, max_y are bounds of map.
	
	int8 map;//map is the array you are wanting information from.

	//direction is the direction the base_link is facing with respect to the world positive x is true and negative x is false
	bool direction;

	float resolution; //resolution is the number of meters per cell in the map.

	Banana_nav(Goal& init_goal,int8 init_map, int init_m_x, int init_m_y, float init_resolution);//Constructor

	~Banana_nav();//Destructor

	//Finds the goal and returns if it found one or not
	bool FindGoal();

	//determines and gives the location of the next spot to look for trees
	bool FindRow();

	//determines if we are done and need to return to base
	bool CheckifDone();

private:
	//function to get cost from occupancy grid given coordinates
	//Param: width ,height is the location of point you want the cost of
	int GetCost(int width, int height);

	//function to get index from occupancy grid given coordinates
	//Param: width ,height is the location of point you want index of
	int GetIndex(int width, int height);

};

#endif /* BANANA_NAV_H_ */



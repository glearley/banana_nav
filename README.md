# banana_nav
Path planning code for UNCA SLAMbot team
Accomplishes three distinct behaviors:
1. Goes through semi-straight row of trees
2. Finds and gets in front of next row of trees
3. Determinies if it is done and returns to start

Topics Subscribed To:
  */move_base/local_costmap/costmap (Move Base's Local Costmap with type OccupencyGrid)
  *Action Topic: MoveBaseAtionClient(Used to Receive notice if goal was successful)

Topics Published To:
  *Action Topic: MoveBaseAtionClient(Used to send goals of type move_base_msgs/goal)

Assumptions:
  * The J5 starts at (0,0) and an empty row is directly in front of the vehicle
  * Semi-straight Row with no aditional obstacles

Questions:
  * when are we done navigating?
  * boundary behavior: finding next row?
  * "I'm Done!" dance?????
  
Misc Nodes:
  *J5 parameters move_base launch file
  * visual odometry reset function (calculates missed odometry?)
  
  

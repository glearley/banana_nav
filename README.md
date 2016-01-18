# banana_nav
Path planning code for UNCA SLAMbot team

Inputs are
  * field_length (positive x)
  * field_width (negitive y)
  * row_width
  
Assumptions:
  * The J5 starts at (0,0) and an empty row is directly in front of the vehicle

Questions:
  * when are we done navigating?
  * boundary behavior: finding next row?
  * "I'm Done!" dance
  
Misc Nodes:
  *J5 parameters move_base launch file
  * visual odometry reset function (calculates missed odometry?)
  
  

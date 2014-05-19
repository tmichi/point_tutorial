point_tutorial
==============

an introduction to point processing for students.

External libraries 
------------------
- OpenGL/GLU
- GLFW (http://www.glfw.org)
- Eigen ( http://eigen.tuxfamily.org ) 
- Windows Kinect SDK


CMake (http://www.cmake.org) is used to build files ( However you do not have to instll it if you create build files manually). 

Programs
-------------
- sample : a sample program for reading and rendering point clouds. type sample data/point.xyz to show example.
- kinect : a sample program for scanning scene by Microsoft Kinect. type [SPACE] key to export point set as "result.xyz"

Assignments to students 
-----------------------
- Estimate the normal vector of each point by using covariance matrix described in Surface reconstruction paper by (Hoppe93).
- Extract planes by RANSAC method. 
- Merge two point clouds by ICP algorithm (Besl92).

Contact
-------
Takashi Michikawa ( michikawa at acm dot org) 

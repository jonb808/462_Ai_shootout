ICS 462, AI for Games
**our added improvements**
  1. will assign roles to tanks for capturing and defending team flags
  2. will improve evasive behavior with zig-zag motion
  
**end of improvements**
===========================================================================
Programming Assignment 2 Solution with modified version of Millington's AI for Games's decision tree code (see LICENSE file for the use license for that original code) from https://github.com/idmillington/aicore

The current version of BZFlag does not allow robots to pick up/drop flags. Download the fixes in robotFlags-2.4.18-v3.zip (http://www2.hawaii.edu/~chin/462/Assignments/robotFlags-2.4.18-v3.zip), unzip it and replace the corresponding files in your src and include directory with these files. You will have to quit bzfs and rebuild it. Likewise with bzflag.

To compile, copy the files RobotPlayer.h, RobotPlayer.cxx, AStarNode.h, AStarNode.cxx, Astar.h, Astar.cxx, dectree.h, and dectree.cxx to your bzflags-2.4.18\bzflag\src\bzflag folder (overwriting the originals of RobotPlayer.h and RobotPlayer.cxx).  Then add AStarNode.h to the bzflag project by right clicking bzflag, selecting "Add > Existing Item" and then find AStarNode.h in your src\bzflag folder.  Likewise add AStarNode.cxx, Astar.h, Astar.cxx, dectree.h and dectree.cxx to the bzflag project. Then build bzflag in Microsoft Visual Studio as usual.

For UNIX-based systems copy the same files to your bzflags-2.4.18/src/bzflag directory.  You will have to edit src/bzflag/Makefile.am to add AStarNode.h, AStarNode.cxx, Astar.h, Astar.cxx, dectree.h, and dectree.cxx to bzflag_SOURCES.

Depending on your OS and compiler, you may have to convert the line endings using a program like dos2unix. Next run autogen.sh and configure again.  Finally compile as usual.

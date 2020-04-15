ICS 462, AI for Games
Programming Assignment 2 Solution with modified version of Millington's AI for Games's decision tree code (see LICENSE file for the use license for that original code) from https://github.com/idmillington/aicore

The current version of BZFlag does not allow robots to pick up/drop flags. Download the fixes in robotFlags-2.4.18-v3.zip (http://www2.hawaii.edu/~chin/462/Assignments/robotFlags-2.4.18-v3.zip), unzip it and replace the corresponding files in your src and include directory with these files. You will have to quit bzfs and rebuild it. Likewise with bzflag.

This solution uses the DOSL( https://github.com/subh83/DOSL ) C++ template implementation of A* search.  Clone or download  and unzip the DOSL GitHub source to your choice of location.  Then in Microsoft Visual Studio, right click on the bzflag project, select Properties and add the DOSL folder as a new entry to "Additional Include Directories" under Configuration Properties, C/C++, General.  If you are using MS Visual Studio for compilation,  rename "ERROR" to "DOSLERROR" in the enum NodeEventType, which appears in the files DSOL\dosl\planners\AStar.tcc, SStar.tcc, and ThetaStar.tcc, in order to avoid a name conflict with an "ERROR" constant already defined in BZFlag. 

To compile, copy the files RobotPlayer.h, RobotPlayer.cxx, myNode.h, myNode.cxx, dectree.h, and dectree.cxx to your bzflags-2.4.18\bzflag\src\bzflag folder (overwriting the originals of RobotPlayer.h and RobotPlayer.cxx).  Then add myNode.h to the bzflag project by right clicking bzflag, selecting "Add > Existing Item" and then find myNode.h in your src\bzflag folder.  Likewise add myNode.cxx, dectree.h, and dectree.cxx to the bzflag project. Then build bzflag in Microsoft Visual Studio as usual.

For UNIX-based systems copy the same files to your bzflags-2.4.2/src/bzflag directory.  You will have to edit src/bzflag/Makefile.am to add AStarNode.h, AStarNode.cxx, dectree.h, and dectree.cxx to bzflag_SOURCES and add the full path to DOSL to src/bzflag/Makefile.am by adding it as a -I argument to the AM_CPPFLAGS line:

AM_CPPFLAGS += $(SDL_CFLAGS) -I /full-path-to/DOSL

Depending on your OS and compiler, you may have to convert the line endings using a program like dos2unix. Next run autogen.sh and configure again.  Finally compile as usual.
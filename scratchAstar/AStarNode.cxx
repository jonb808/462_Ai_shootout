#include "AStarNode.h"
#include <math.h>
#include "common.h"
#include "BZDBCache.h" // needed for worldSize
#include "World.h"
#include "playing.h" // needed for controlPanel


AStarNode::AStarNode(void)
{
	status = unseen;
}

AStarNode::~AStarNode(void)
{
}

// Takes a bzflag world location and returns the closest AStarNode
// if that node is not isAccessible (the location is next to the world boundaries or a building and
// the closest node is outside the world or inside the building), then return a neighbor node that isAccessible
AStarNode::AStarNode(const float location[3])
{
	x = (location[0] > 0.0) ? (int)floor(location[0]/SCALE + 0.5f) : (int)ceil(location[0]/SCALE - 0.5f);
	y = (location[1] > 0.0) ? (int)floor(location[1]/SCALE + 0.5f) : (int)ceil(location[1]/SCALE - 0.5f);
	if (AStarNode::isAccessible(x, y)) return;
  int originalX = x, originalY = y;
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
      x = originalX + a;
      y = originalY + b;
			if (AStarNode::isAccessible(x, y)) return;
		}
	char buffer[128];
	sprintf_s(buffer, "***AStarNode: could not find any isAccessible node for (%f, %f, %f)***", location[0], location[1], location[2]);
	controlPanel->addMessage(buffer);
}
AStarNode::AStarNode(int xi, int yi)
{
	x = xi;
	y = yi;
}

// returns true if the location (x, y) is inside the world boundaries and not in a building
bool AStarNode::isAccessible(int x, int y)
{
	float pos0 = x*SCALE;
	float pos1 = y*SCALE;
	const float pos[3] = { x*SCALE, y*SCALE, 0.0f };
	// check for world boundaries
	if (pos[0]<-BZDBCache::worldSize/2 || pos[0]>BZDBCache::worldSize/2 ||
		pos[1]<-BZDBCache::worldSize/2 || pos[1]>BZDBCache::worldSize/2)
		return false;
	// if not inside an obstacle
	return !World::getWorld()->inBuilding(pos, BZDBCache::tankRadius / 2, BZDBCache::tankHeight);
}

// member function version of above
bool AStarNode::isAccessible(void)
{
  if (accessibleP == unknown) {
    accessibleP = isAccessible(getX(), getY())? accessible : inaccessible;
  }
  return (accessibleP == accessible);
}

// returns the Euclidean distance of the AStarNode to the goalNode
double AStarNode::getHeuristic(const AStarNode * goalNode) const
{
	return hypotf(x - goalNode->getX(), y - goalNode->getY());
}

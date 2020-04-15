#pragma once

#ifndef	BZF_ASTARNODE_H
#define	BZF_ASTARNODE_H

//#include <vector>
//using std::vector;
#include "BZDBCache.h"

#define SCALE	BZDBCache::tankRadius
// A node of the A* search graph

// status is unseen (not yet visited), on the open list or on the closed list
enum AStarStatus {unseen, open, closed};

enum AStarAccessible { unknown, accessible, inaccessible };

// Stores a node in the A* search graph
class AStarNode
{
public:
	AStarNode(void);
	~AStarNode(void);

	bool operator==(const AStarNode& n) const { return (x==n.x && y==n.y); };
	bool operator<(const AStarNode& n) const { return (getTotalCost() < n.getTotalCost()); };
	AStarNode(const float location[3]);
	AStarNode(int xi, int yi);
	bool isAccessible(void);
	static bool isAccessible(int x, int y);
	double getHeuristic(const AStarNode * goalNode) const;
	inline int AStarNode::getX(void) const { return x; }
	inline int AStarNode::getY(void) const { return y; }
	inline void AStarNode::setX(int newX) { x = newX; }
	inline void AStarNode::setY(int newY) { y = newY; }
	inline int AStarNode::getPrevX(void) const { return prevX; }
	inline int AStarNode::getPrevY(void) const { return prevY; }
	inline void AStarNode::setPrevX(int newX) { prevX = newX; }
	inline void AStarNode::setPrevY(int newY) { prevY = newY; }
	inline float AStarNode::getScaledX(void) const { return x * SCALE; }
	inline float AStarNode::getScaledY(void) const { return y * SCALE; }
	inline double AStarNode::getCostSoFar(void) const { return costSoFar; }
	inline void AStarNode::setCostSoFar(double newCostSoFar) { costSoFar = newCostSoFar; }
	inline double AStarNode::getTotalCost(void) const { return totalCost; }
	inline void AStarNode::setTotalCost(double newTotalCost) { totalCost = newTotalCost; }
	inline AStarStatus AStarNode::getStatus(void) const { return status; }
	inline void AStarNode::setStatus(AStarStatus newStatus) { status = newStatus; }
    inline AStarAccessible AStarNode::getAccessibleP(void) const { return accessibleP; }
    inline void AStarNode::setAccessibleP(AStarAccessible newAccessibleP) { accessibleP = newAccessibleP; }
private:
	int x, y;
	int prevX, prevY;
	double costSoFar;
	double totalCost;
	AStarStatus status;
    AStarAccessible accessibleP;
};


#endif // BZF_ASTARNODE_H

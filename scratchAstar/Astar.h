#pragma once
/* system interface headers */
#include <vector>
using std::vector;
#include <set>
using std::set;
#include "AStarNode.h"

#define indexShift (int)(floor(BZDBCache::worldSize / (2 * SCALE) + 1.5f))
#define closedArraySize (2 * (int)(floor(BZDBCache::worldSize / (2 * SCALE) + 1.5f)) + 1)
#define closedArrayFixedSize 200

// define pless function as a binary operator that takes pointers
// to any type and returns the comparison of the objects pointed to
// and when the objects have the same value, compare the pointer values
// This is so that multiset will only consider two elements to be the same
// if and only if they have the same object values and point to the same object
template<typename Type, typename Compare = std::less<Type> >
struct pless : public std::binary_function<Type *, Type *, bool> {
	bool operator()(const Type *x, const Type *y) const
	{
		return (Compare()(*x, *y) ? true : (Compare()(*y, *x) ? false : x<y));
	}
};

// Stores the local data needed for A* Search
class AStarGraph {
public:
	AStarGraph::AStarGraph() {}
	AStarGraph::AStarGraph(const float startPos[3], const float goalPos[3]);
	static void		aStarSearch(const float startPos[3], const float goalPos[3], vector< AStarNode > & path);
	inline AStarNode *		getRecord(AStarNode n) {
		return &closedArray[n.getX() + indexShift][n.getY() + indexShift];
	}
	inline AStarNode *		getRecord(int x, int y) {
		return &closedArray[x + indexShift][y + indexShift];
	}

private:
	AStarNode * startNode;
	AStarNode * goalNode;
	//int indexShift;
        static bool closedArrayInitP;
	static AStarNode closedArray[closedArrayFixedSize][closedArrayFixedSize];
	set<AStarNode *, pless<AStarNode> > openQueue;
	void					startAStar(vector< AStarNode > & path);
	vector<AStarNode *>		getSuccessors(AStarNode * node);
	inline void AStarGraph::addToQueue(AStarNode * neighborNode, AStarNode * currentNode, double incrementalCost) {
		neighborNode->setStatus(open);
		neighborNode->setCostSoFar(currentNode->getCostSoFar() + incrementalCost);
		neighborNode->setTotalCost(neighborNode->getCostSoFar() + neighborNode->getHeuristic(goalNode));
		neighborNode->setPrevX(currentNode->getX());
		neighborNode->setPrevY(currentNode->getY());
		openQueue.insert(neighborNode);
	}
};

#include "Astar.h"
#include "BZDBCache.h" // needed for worldSize
#include "playing.h" // needed for controlPanel

//#define TRACE_ASTAR

bool AStarGraph::closedArrayInitP = false;
AStarNode AStarGraph::closedArray[closedArrayFixedSize][closedArrayFixedSize];

// Initialize A* Search graph with starting position startPos and goal position goalPos (bzflag locations)
AStarGraph::AStarGraph(const float startPos[3], const float goalPos[3])
{
  if (closedArraySize > closedArrayFixedSize) {
    char buffer[128];
    sprintf_s(buffer, "***Astar.cpp: closedArrayFixedSize is too small, needs to be %i***",
      closedArraySize);
    controlPanel->addMessage(buffer);
  }
  //memset(&closedArray, 0, sizeof closedArray);
  if (!closedArrayInitP) {
    for (int i = 0; i < closedArrayFixedSize; ++i) {
      for (int j = 0; j < closedArrayFixedSize; ++j) {
        closedArray[i][j].setX(i - indexShift);
        closedArray[i][j].setY(j - indexShift);
        closedArray[i][j].setAccessibleP(unknown);
      }
    }
    closedArrayInitP = true;
  }

  for (int i = 0; i < closedArrayFixedSize; ++i) {
    for (int j = 0; j < closedArrayFixedSize; ++j) {
      closedArray[i][j].setStatus(unseen);
    }
  }

	AStarNode sNode = AStarNode(startPos);
	startNode = getRecord(sNode);
	startNode->setX(sNode.getX());
	startNode->setY(sNode.getY());
	AStarNode gNode = AStarNode(goalPos);
	goalNode = getRecord(gNode);
	goalNode->setX(gNode.getX());
	goalNode->setY(gNode.getY());
}

// return a path (a vector of AStarNode with the goal node first [0] and the start node last [path->size() - 1])
// that is the result of running A* search from startPos to goalPos (bzflag locations)
void		AStarGraph::aStarSearch(const float startPos[3], const float goalPos[3], vector< AStarNode > & path)
{
	AStarGraph myAStarGraph(startPos, goalPos);
	if (!path.empty()) path.clear();
	myAStarGraph.startAStar(path);
	if (path.empty()) {
		char buffer[128];
		sprintf_s(buffer, "***AStarGraph::aStarSearch: could not find a path from (%f, %f) to (%f, %f)***",
			startPos[0], startPos[1], goalPos[0], goalPos[1]);
		controlPanel->addMessage(buffer);
	}
#ifdef TRACE_ASTAR
	char buffer[128];
	sprintf(buffer, "A* search from (%f, %f) to (%f, %f) found path, size = %d",
		startPos[0], startPos[1], goalPos[0], goalPos[1], path.size());
	controlPanel->addMessage(buffer);
	for (int a = 0; a < path.size(); a++) {
		sprintf(buffer, "[%f, %f]; ", path[a].getScaledX(), path[a].getScaledY());
		controlPanel->addMessage(buffer);
	}
	controlPanel->addMessage(" ]\n\n");
#endif
}

// helper function for above that runs the actual loop of A* search
void		AStarGraph::startAStar(vector< AStarNode > & path)
{
	const double distanceArray[8] = { M_SQRT2, 1, M_SQRT2, 1, 1, M_SQRT2, 1, M_SQRT2 };
	startNode->setCostSoFar(0.0f);
	startNode->setStatus(open);
	startNode->setTotalCost(startNode->getHeuristic(goalNode));
	openQueue.insert(startNode);
	while (!openQueue.empty()) {
		AStarNode * currentNode = *openQueue.begin();
		if (currentNode == goalNode) break;
		openQueue.erase(currentNode);
		int neighborcount = 0;
		for (auto &neighborNode : getSuccessors(currentNode)) {
			double incrementalCost = distanceArray[neighborcount++];
			bool accessible = neighborNode->isAccessible();
			if (!accessible) continue; // skip if not accessible
			// uncomment out the next 4 lines to return as soon as a goal node is seen
			// this would not be true A* and returns a good enough, not the best, path to goal
			// For bzflag, this efficiency hack makes no measureable difference
			//if (neighborNode == goalNode) {
			//	addToQueue(neighborNode, currentNode, incrementalCost);
			//	break;
			//}
			// check if neighborNode is closed
			if (neighborNode->getStatus() == closed) {
				if (neighborNode->getCostSoFar() <= (currentNode->getCostSoFar() + incrementalCost)) continue;
				// closedNode is more expensive, so remove it from closedArray and add it to openQueue
				neighborNode->setStatus(open);
				addToQueue(neighborNode, currentNode, incrementalCost);
			}
			// check if neighborNode is open
			else if (neighborNode->getStatus() == open) {
				if (neighborNode->getCostSoFar() <= (currentNode->getCostSoFar() + incrementalCost)) continue;
				// openNode is more expensive, so replace it with neighborNode
				openQueue.erase(neighborNode);
				addToQueue(neighborNode, currentNode, incrementalCost);
			}
			else { // neighborNode has not been visited before, so add to openQueue
				addToQueue(neighborNode, currentNode, incrementalCost);
			}
		}
		currentNode->setStatus(closed);
	}

	if (openQueue.empty()) {
		char buffer[128];
		sprintf_s(buffer, "***AStarGraph::startAStar: empty openQueue (%i, %i) to (%i, %i)***",
			startNode->getX(), startNode->getY(), goalNode->getX(), goalNode->getY());
		controlPanel->addMessage(buffer);
	}
	else {
		AStarNode* nextNodePtr = *openQueue.begin();
		path.clear();
		do {
			path.push_back(*nextNodePtr);
			nextNodePtr = getRecord(nextNodePtr->getPrevX(), nextNodePtr->getPrevY());
		} while (nextNodePtr != startNode);
	}
}

// return the neighbors of the AStarNode as a vector { NW, W, SW, N, S, NE, W, NW } of pointers
// into closedArray
vector<AStarNode *> AStarGraph::getSuccessors(AStarNode * node)
{
	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	vector<AStarNode *> neighbors(8);
	int x = node->getX(), y = node->getY(), count = 0;
	for (int a = -1; a <= 1; a++)
		for (int b = -1; b <= 1; b++) {
			if (a == 0 && b == 0) continue;
			neighbors[count] = getRecord(x + a, y + b);
			//neighbors[count]->setX(x + a);
			//neighbors[count]->setY(y + b);
			++count;
		}
	return neighbors;
}

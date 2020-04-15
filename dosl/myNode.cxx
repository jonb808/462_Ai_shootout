#include <stdio.h>
#include <vector>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "common.h"
#include "World.h"
#include "playing.h" // needed for controlPanel
#include "ControlPanel.h"
#include "myNode.h"

bool myNode::isAccessible(int x, int y)
{
    float pos0 = x * SCALE;
    float pos1 = y * SCALE;
    const float pos[3] = { x * SCALE, y * SCALE, 0.0f };
    // check for world boundaries
    if (pos[0]<-BZDBCache::worldSize / 2 || pos[0]>BZDBCache::worldSize / 2 ||
        pos[1]<-BZDBCache::worldSize / 2 || pos[1]>BZDBCache::worldSize / 2)
        return false;
    // if not inside an obstacle
    return !World::getWorld()->inBuilding(pos, BZDBCache::tankRadius, BZDBCache::tankHeight);
}

// static member function version of above
bool myNode::isAccessible()
{
    return isAccessible(x, y);
}
myNode::myNode(const float location[3])
{
    x = (location[0] > 0.0) ? (int)floor(location[0] / SCALE + 0.5f) : (int)ceil(location[0] / SCALE - 0.5f);
    y = (location[1] > 0.0) ? (int)floor(location[1] / SCALE + 0.5f) : (int)ceil(location[1] / SCALE - 0.5f);
    if (isAccessible(x, y)) return;
    int originalX = x, originalY = y;
    for (int a = -1; a <= 1; a++)
        for (int b = -1; b <= 1; b++) {
            if (a == 0 && b == 0) continue;
            x = originalX + a;
            y = originalY + b;
            if (isAccessible(x, y)) return;
        }
    char buffer[128];
    sprintf_s(buffer, "***AStarNode: could not find any isAccessible node for (%f, %f, %f)***", location[0], location[1], location[2]);
    controlPanel->addMessage(buffer);
}

void searchProblem::getSuccessors(myNode& n, std::vector<myNode>* const s, std::vector<double>* const c)
{
    // This function should account for obstacles, constraints and size of environment.
    myNode tn;
    //s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
    for (int a = -1; a <= 1; a++)
        for (int b = -1; b <= 1; b++) {
            if (a == 0 && b == 0) continue;
            tn.setX(n.getX() + a);
            tn.setY(n.getY() + b);
            if (tn.isAccessible()) {
                s->push_back(tn);
                if (a == 0 || b == 0)
                    c->push_back(1);
                else
                    c->push_back(M_SQRT2);
            }
        }
}



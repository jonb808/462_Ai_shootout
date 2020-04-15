#define _DOSL_VERBOSE_LEVEL -1
#include <dosl/dosl>
#include "BZDBCache.h" // needed for worldSize, tankRadius
// DOSL headers:
#define SCALE	BZDBCache::tankRadius
// A node of the A* search graph
class myNode : public AStar::Node<myNode, double>
{
public:
    int x, y; // (x,y) coordinates defining a point on plane.
    inline int getX(void) const { return x; }
    inline int getY(void) const { return y; }
    inline void setX(int newX) { x = newX; }
    inline void setY(int newY) { y = newY; }
    inline float getScaledX(void) const { return x * SCALE; }
    inline float getScaledY(void) const { return y * SCALE; }

    myNode() : x(0), y(0) { }
    myNode(int xx, int yy) : x(xx), y(yy) { }
    myNode(const float location[3]);
    // The comparison operator must be defined for the node type.
    bool operator==(const myNode& n) const { return (x == n.x && y == n.y); }
    // An efficint hash function, 'getHashBin', for node type is desired, but is optional.
    int getHashBin(void) { return (abs(((int)x >> 4) + ((int)y << 3) + ((int)x << 4) + ((int)y >> 3))); }

    bool isAccessible(int x, int y);
    bool isAccessible();
};

class searchProblem : public AStar::Algorithm<searchProblem, myNode, double>
{
public:
    // user-defined problem parameters:
    myNode a;
    myNode start_node;
    myNode goal_node;
    // Constructors, if any
    searchProblem() { }
    void init(myNode& start, myNode& goal) { start_node = start;  goal_node = goal; }
    /* Prototype for 'AStar::Algorithm<>::getStartNodes':
       std::vector<NodeType> getStartNodes (void);
       Description: Should return the list of vertices(s) to start the search with. */
    std::vector<myNode> searchProblem::getStartNodes(void) {
        std::vector<myNode> startNodes;
        //for (int a = 0; a < 1; ++a) {
        //    myNode tn(0, 0); // start node
        //    startNodes.push_back(tn);
        //}
        startNodes.push_back(start_node);
        return (startNodes);
    }
    /* Prototype for 'AStar::Algorithm<>::stopSearch':
       bool stopSearch (NodeType& n);
       Description: Determines whether to stop the search when 'n' is being expanded.
       Optional -- If not provided, search will terminate only when heap is empty. */
    bool stopSearch(myNode& n) {
        return (n == goal_node);
    }
    // -----------------------------------------------------------
    /* The following functions are use by the base class 'AStar::Algorithm' to determine
       graph structure and search parameters. */

       /* Prototype for 'AStar::Algorithm<>::getSuccessors':
              void getSuccessors
                  (NodeType& n, std::vector<NodeType>* const s, std::vector<CostType>* const s);
          Description: Takes in a vertex, n, and returns its neighbors/successors, s,
                       and the costs/distances of the edges, c. This defines graph connectivity. */
    void getSuccessors(myNode& n, std::vector<myNode>* const s, std::vector<double>* const c);
};

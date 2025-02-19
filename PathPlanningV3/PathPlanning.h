#ifndef PathPlanning_H
#define PathPlanning_H

#define MAX_PATH_SIZE 20
#define INF 9999

// External Functions
extern bool detectObstacle();
extern bool detectOuterObstacle();
extern void setColour(int r, int g, int b);
extern void driveMotor(int left, int right);

extern void left();
extern void right();
extern void reverse();

// File specific Variables
extern bool atNode;
extern bool forwardDirection;   //Start with forward direction
const int nodeCount = 8; // Number of nodes

// Adjacency Matrix
extern int weightMatrix[nodeCount][nodeCount];
extern int path[MAX_PATH_SIZE];  // Final path with virtual nodes
extern int pathLength;  // Size of the updated path
extern int updatedPath[MAX_PATH_SIZE];
extern int updatedPathLength;

extern int pathIndex;
extern int lastNode;

extern int current;
extern int next;

// Obstacle re-routing variables
extern int tempPath[MAX_PATH_SIZE];
extern int tempPathLength;
extern int reRouteIndex;
extern bool reRouteActive;
 
// Re-routing variables
extern int storeWeight;
extern int storeCurrent;
extern int storeNext;


// Get next direction
int getDirection(int currentNode, int lastNode, int nextNode);

// Handle directions for junctions
int getJunctionDirection(int currentNode, int lastNode, int nextNode);

// Choose Path based on direction
void choosePath(int direction);

// Process path path array and keep track of position 
void processPath(int currentPath[], int &index, int pathLength, bool isTempRoute);

// Find min distance between nodes
int findMinDistance(int distances[], bool visited[]);

//Find Dijkstra shortest path and update path array 
void shortestPath(int startNode, int endNode, int tempPath[], int &tempPathLength);

// calculate shortest path for entire route
void computePath();

// re calculate route between current node and next node if obstacle detected.
bool reRoute(int current, int next);

#endif
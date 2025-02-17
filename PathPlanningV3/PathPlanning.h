#ifndef PathPlanning_H
#define PathPlanning_H

extern bool forwardDirection;
extern bool atNode;
extern int nodeCount;
extern int baseSpeed;
extern int forwardDelay;
extern int lastNode;

extern int tempPathLength;
extern int reRouteIndex;
extern int path[];
extern int pathLength;
extern int updatedPath[];
extern int updatedPathLength;

extern bool detectObstacle();
extern void sendPosition();
extern void setColour();



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
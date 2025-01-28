const int nodeCount = 8;
const int adjacencyList[nodeCount][3] = {
  { 4, 6, -1 },     // Node 0: {Back=4, Straight=6, Left=NONE}
  { 6, 7, -1 },     // Node 1: {Back=6, Straight=7, Left=NONE}
  { 6, 3, -1 },     // Node 2: {Back=6, Straight=3, Left=NONE}
  { 2, 7, -1 },     // Node 3: {Back=2, Straight=7, Left=NONE}
  { 7, 0, -1 },     // Node 4: {Back=7, Straight=0, Left=NONE}
  { -1, -1, -1 },   // Node 5: {Back=NONE, Straight=NONE, Left=NONE}
  { 0, 2, 1 },      // Node 6: {Back=0, Straight=2, Left=1}
  { 3, 4, 1 }       // Node 7: {Back=3, Straight=4, Left=1}
};
//-------------------------------------------------------------------------------
const int INF = 10000; // A large number to represent infinity

int startNode = 0; // Change as needed
int endNode = 7;   // Change as needed

int globalPath[nodeCount]; // Global array to store the shortest path
int globalPathLength = 0;  // Global variable to store the path length


void setup() {
}

void loop() {
  // Nothing to do here
}

//------------------------------------------------------------------
// Function to find the node with the smallest distance that hasn't been visited
int findMinDistance(int distances[], bool visited[]) {
  int minDistance = INF;
  int minIndex = -1;

  for (int i = 0; i < nodeCount; i++) {
    if (!visited[i] && distances[i] < minDistance) {
      minDistance = distances[i];
      minIndex = i;
    }
  }

  return minIndex;
}

// Function to compute the shortest path using Dijkstra's algorithm
void shortestPath(int startNode, int endNode) {
  int distances[nodeCount]; // Array to store the shortest distance from startNode to each node
  bool visited[nodeCount];  // Array to track visited nodes
  int previous[nodeCount];  // Array to store the path

  // Initialize distances, visited, and previous arrays
  for (int i = 0; i < nodeCount; i++) {
    distances[i] = INF;
    visited[i] = false;
    previous[i] = -1;
  }
  distances[startNode] = 0;

  // Dijkstra's algorithm
  for (int i = 0; i < nodeCount - 1; i++) {
    int currentNode = findMinDistance(distances, visited);

    if (currentNode == -1) {
      break; // All reachable nodes have been visited
    }

    visited[currentNode] = true;

    // Update distances to neighbors of the current node
    for (int j = 0; j < 3; j++) {
      int neighbor = adjacencyList[currentNode][j];

      if (neighbor != -1 && !visited[neighbor]) {
        int newDistance = distances[currentNode] + 1; // Assuming uniform edge weight of 1

        if (newDistance < distances[neighbor]) {
          distances[neighbor] = newDistance;
          previous[neighbor] = currentNode;
        }
      }
    }
  }

  // Store the shortest path in the global array
  globalPathLength = 0;
  for (int at = endNode; at != -1; at = previous[at]) {
    globalPath[globalPathLength++] = at;
  }

  if (distances[endNode] == INF) {
    Serial.println("No path found.");
    return;
  }
}

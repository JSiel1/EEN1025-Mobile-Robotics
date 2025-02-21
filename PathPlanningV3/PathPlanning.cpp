#include <Arduino.h>
#include "PathPlanning.h"
#include "serverCommunication.h"
#include "motorControl.h"
#include "settings.h"

// PathPlanning specific variables.
bool atObstacle = false;
bool atNode = false;
bool forwardDirection = true;   //Start with forward direction

// Adjacency Matrix
int weightMatrix[8][8] = {
  //   0    1     2     3    4    5     6     7
  {    0, INF,  INF,  INF,   1, INF,    2, INF },    // Node 0: connects to 4 and 6
  { INF,    0,  INF,  INF, INF, INF,    2,   1 },    // Node 1: connects to 6 and 7
  { INF,  INF,    0,    2, INF, INF,    1, INF },    // Node 2: connects to 3 and 6
  { INF,  INF,    2,    0, INF, INF,  INF,   1 },    // Node 3: connects to 2 and 7
  {   2,  INF,  INF,  INF,   0, INF,  INF,   1 },    // Node 4: connects to 0 and 7
  { INF,  INF,  INF,  INF, INF,   0,  INF,   1 },    // Node 5: isolated
  {   1,    2,    2,  INF, INF, INF,    0, INF },    // Node 6: junction (nodes 0,1,2)
  { INF,    2,  INF,    1,   2,   1,  INF,   0 }     // Node 7: junction (nodes 1,3,4,5)
};

int path[MAX_PATH_SIZE];  // Final path with virtual nodes
int pathLength = 0;  // Size of the updated path
int updatedPath[MAX_PATH_SIZE];
int updatedPathLength = 0;

int pathIndex = 0;
int lastNode = -1;

int current = -1;
int next = -1;

// Obstacle re-routing variables
int tempPath[MAX_PATH_SIZE];
int tempPathLength = 0;
int reRouteIndex = 0;
bool reRouteActive = false;

// Re-routing variables
int storeWeight = -1;
int storeCurrent = -1;
int storeNext = -1;


//-------------------------------------------------------------
//-----------------Path Following Logic------------------------
//------------------------------------------------------------

// Get next direction
int getDirection(int currentNode, int lastNode, int nextNode) {
  // At the starting position, assume going straight.
  if (lastNode == -1)
    return (forwardDirection) ? 1 : 0;
    
  // If more than two connections exist, treat it as a junction.
  if (currentNode == 6 || currentNode == 7)
    return getJunctionDirection(currentNode, lastNode, nextNode);
    
  // For nodes with only two connections, if the next node equals the last node, indicates a 180° turn.
  if (nextNode == lastNode)
    return 0;
    
  // Otherwise, go straight.
  return 1;
}

// Handle directions for junctions
int getJunctionDirection(int currentNode, int lastNode, int nextNode) {
  // Node 6 mapping: connected to nodes 0, 1, and 2.
  if (currentNode == 6) {
    if (lastNode == 1) {
      if (nextNode == 2) return 2; // left -> node 2
      if (nextNode == 0) return 3; // right -> node 0
      if (nextNode == 1) return 0; // back (180° turn)
    } else if (lastNode == 2) {
      if (nextNode == 0) return 1; // straight -> node 0
      if (nextNode == 1) {
        forwardDirection = !forwardDirection;
        return 3; // right (with flip)
      }
      if (nextNode == 2) return 0; // back
    } else if (lastNode == 0) {
      if (nextNode == 1) return 2; // left -> node 1
      if (nextNode == 2) return 1; // straight -> node 2
      if (nextNode == 0) return 0; // back
    }
  }
  
  // Node 7 mapping: connected to nodes 1, 3, and 4.
  if (currentNode == 7) {
    if (lastNode == 1) {
      if (nextNode == 1) return 0; // back -> node 1
      if (nextNode == 3) {
        forwardDirection = !forwardDirection;
        return 3; // right -> node 3
      }
      if (nextNode == 4) return 2; // left -> node 4
      if (nextNode == 5) return 1; // striaght -> node 5
    } else if (lastNode == 4) {
      if (nextNode == 1) {
        forwardDirection = !forwardDirection; // flip direction
        return 3; // right -> node 1
      }
      if (nextNode == 3) return 1; // straight -> node 3
      if (nextNode == 4) return 0; // back -> node 4
      if (nextNode == 5) return 2; // left -> node 5
    } else if (lastNode == 3) {
      if (nextNode == 1) {
        forwardDirection = !forwardDirection;
        return 2; // left
      }
      if (nextNode == 1) return 2; // left -> node 1
      if (nextNode == 3) return 0; // back -> node 3
      if (nextNode == 4) return 1; // straight -> node 4
      if (nextNode == 5) return 3; // right -> node 5
    }
  }
  return -1; // error: mapping not found
}

// Choose Path based on direction
void choosePath(int direction){
  switch (direction) {
    case 0:                              // reverse
      Serial.println("Turning Around");
      reverse();                         // 180-degree turn
      break;
    case 1:                              // Straight
      Serial.println("Going Straight");
      driveMotor(baseSpeed, baseSpeed);
      delay(forwardDelay);                             
      break;
    case 2:                              // Left
      Serial.println("Turning Left");
      left();
      break;
    case 3:
      Serial.println("Turning Right");
      right();                          // Right
      if (forwardDirection) {
        forwardDirection = false;
      }
      break;
    default:
      Serial.println("Error: Direction Invalid");
      driveMotor(0, 0);
      break;
  }
}

// Process path path array and keep track of position 
void processPath(int currentPath[], int &index, int pathLength, bool isTempRoute) {
  current = currentPath[index];
  next = currentPath[index + 1];
  
  // Obstacle detection & temporary re-routing.
  if (!isTempRoute && atObstacle) {
    // Update position index if re-routing
    if (index > 0) {
      current = currentPath[index - 1];
      next = currentPath[index];
    } 

    if (!reRoute(current, next)) {
      Serial.println("Error in Re-routing: No alternate route possible.");
      driveMotor(0, 0);
      return;
    }
    return; // Exit and let the next loop iteration process the temporary path
  }

  //Return if not at node
  if (!atNode) {
    return;
  }
  
  driveMotor(0, 0); // Stop the robot
  driveMotor(80, 80); // Drive forward at low speed
  delay(forwardDelay);          // Move slightly forward to cross the line
  driveMotor(0, 0);   // Stop again

  if (index < pathLength - 1) {
    // Only send position on nodes and not during re-routing    
    if (current != 6 && current != 7 && !isTempRoute) {
      sendPosition(current);
    }

    int turnCode = getDirection(current, lastNode, next);
    Serial.print("At node ");
    Serial.print(current);
    Serial.print(" -> next node ");
    Serial.print(next);
    Serial.print(" : Turn code = ");
    Serial.println(turnCode);
    
    // Perform action based on turn code
    choosePath(turnCode);
    
    //Parking logic
    if (next == 5) {
      while (!detectObstacle()){
        Serial.println("Waiting for wall");

        //drive straight at wall
        driveMotor(baseSpeed + 5, baseSpeed);
      }

      //Updated final position and stop
      driveMotor(0,0);
      sendPosition(5);
      while (1) {
        driveMotor(0,0);
        setColour(0, 255, 0);
        delay(300);
        setColour(0,0,0);
        delay(300);
      }
    }
    // Increment the global path index to move to the next segment.
    index++;
    lastNode = current;
  } else if (!isTempRoute) {
    // If we have reached the end of the path, indicate completion.
    Serial.println("Finished path.");
    sendPosition(updatedPath[updatedPathLength - 1]);
    // Enter an infinite loop after finishing.
    while (true) {
      driveMotor(0,0);
      setColour(0, 255, 0);
      delay(300);
      setColour(0,0,0);
      delay(300);
    }
  } else {
    Serial.println("Temp Route exit ");
    Serial.print("Temp index: ");
    Serial.println(reRouteIndex);
    Serial.print("Path Length: ");
    Serial.println(tempPathLength);
    delay(2000);    //debug
  }
}

//-------------------------------------------------------------
//-----------------Shortest Path Calculation-------------------
//-------------------------------------------------------------

// Find min distance between nodes
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

//Find Dijkstra shortest path and update path array 
void shortestPath(int startNode, int endNode, int tempPath[], int &tempPathLength) {
  int distances[nodeCount];
  bool visited[nodeCount];
  int previous[nodeCount];

  for (int i = 0; i < nodeCount; i++) {
    distances[i] = INF;
    visited[i] = false;
    previous[i] = -1;
  }
  distances[startNode] = 0;

  for (int i = 0; i < nodeCount - 1; i++) {
    int currentNode = findMinDistance(distances, visited);
    if (currentNode == -1) break;
    visited[currentNode] = true;

    for (int neighbor = 0; neighbor < nodeCount; neighbor++) {
      if (weightMatrix[currentNode][neighbor] != INF && !visited[neighbor]) {
        int newDistance = distances[currentNode] + weightMatrix[currentNode][neighbor];
        if (newDistance < distances[neighbor]) {
          distances[neighbor] = newDistance;
          previous[neighbor] = currentNode;
        }
      }
    }
  }
  
  tempPathLength = 0;
  for (int at = endNode; at != -1; at = previous[at]) {
    tempPath[tempPathLength++] = at;
  }
  
  if (distances[endNode] == INF) {
    Serial.println("No path found.");
    tempPathLength = 0;
  }
}

// calculate shortest path for entire route
void computePath() {
  updatedPathLength = 0;
  
  // If there is no or only one node in the route, just copy it over.
  if (pathLength <= 0)
    return;
  if (pathLength == 1) {
    updatedPath[0] = path[0];
    updatedPathLength = 1;
    return;
  }
  
  // Temporary array to hold the shortest path between two nodes.
  int tempPath[MAX_PATH_SIZE];
  int tempPathLength;
  
  // Iterate over each consecutive pair in the global route.
  for (int i = 0; i < pathLength - 1; i++) {
    // Compute the shortest path from path[i] to path[i+1]
    shortestPath(path[i+1], path[i], tempPath, tempPathLength);

    // If no path was found, print an debug and reset updatedPathLength.
    if (tempPathLength == 0) {
      Serial.print("No path found between ");
      Serial.print(path[i]);
      Serial.print(" and ");
      Serial.println(path[i + 1]);
      updatedPathLength = 0;
      return;
    }
        
    // append the path
    int startIndex = (i == 0) ? 0 : 1;
    for (int j = startIndex; j < tempPathLength; j++) {
      updatedPath[updatedPathLength++] = tempPath[j];
    }
  }  

  // check if next position is behind starting position
  if (updatedPathLength > 1) {  // Ensure at least two positions exist
    int start = updatedPath[0];
    int next = updatedPath[1];

    // Check if which node has higher weighting to indicate if starting in reverse direction
    if (weightMatrix[next][start] != INF && weightMatrix[start][next] != INF) {
      Serial.print(weightMatrix[next][start]);
      Serial.print(" > ");
      Serial.println(weightMatrix[start][next]);
      if (weightMatrix[next][start] > weightMatrix[start][next]) {
        forwardDirection = false;  // Flip direction
        Serial.println("Starting Direction flipped");
      }
    }
  }

  Serial.print("Shortest path: ");
  for (int i = updatedPathLength - 1; i >= 0; i--) {
    Serial.print(updatedPath[i]);
    if (i > 0) Serial.print(" -> ");
  }
  Serial.println();
}

// re calculate route between current node and next node if obstacle detected.
bool reRoute(int current, int next) {
  Serial.println("Obstacle detected! Calculating temporary route...");

  // stop the robot before the obstacle
  driveMotor(0,0);

  //reverse slightly
  driveMotor(-40, -40);
  delay(50);

  // Backup the original weight before removing the connection
  storeWeight = weightMatrix[current][next];
  storeCurrent = current;
  storeNext = next;

  // Temporarily remove the direct connection
  weightMatrix[current][next] = INF;
  weightMatrix[next][current] = INF;

  // Compute temporary route
  int newPath[MAX_PATH_SIZE];
  int newPathLength = 0;
  shortestPath(next, current, newPath, newPathLength);    //changed this

  if (newPathLength == 0) {
    Serial.println("Re-route Error: No alternate path found! Stopping robot.");
    return false;  // Indicate failure
  }

  // Copy the new path into the tempPath array AND DEBUG
  for (int i = 0; i < newPathLength; i++) {
    tempPath[i] = newPath[i];
  }
  tempPathLength = newPathLength;

  // DEBUG
  Serial.print("Re-route: ");
  for (int j = 0; j < tempPathLength; j++) {
    Serial.print(tempPath[j]);
    if (j < tempPathLength - 1) Serial.print(" -> ");
  }
  Serial.println("");

  // Activate temporary routing mode
  reRouteActive = true;
  reRouteIndex = 0;

  lastNode = next;

  //turn the robot around.
  reverse();

  return true;  // Indicate success
}

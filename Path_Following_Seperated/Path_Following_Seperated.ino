// Adjacency Matrix: -1 means no path
const int nodeCount = 8; // Number of nodes
const int adjacencyList[nodeCount][3] = {
  { 4, 6, -1 }, // Node 0: Back=-4, Straight=6, Left=NONE
  { 6, 7, -1 },  // Node 1: Back=6, Straight=7, Left=NONE
  { 6, 3, -1 }, // Node 2: Back=6, Straight=3, Left=NONE
  { 2, 7, -1 }, // Node 3: Back=2, Straight=7, Left=NONE
  { 7, 0, -1 }, // Node 4: Back=7, Straight=0, Left=NONE
  { -1, -1, -1 },  // Node 5: Back=NONE, Straight=NONE, Left=NONE
  { 0, 2, 1 },   // Node 6: Back=0, Straight=2, Left=1
  { 3, 4, 1 }   // Node 7: Back=3, Straight=4, Left=1
};

// Define the path to follow (e.g., 0 → 6 → 1 → 5)
const int path[] = {0, 6, 1, 5};
const int pathLength = sizeof(path) / sizeof(path[0]);

//-----------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
//-----------------------------------------------------------------------

// Function to get the next direction (Reverse=0, Forward=1, Left=2)
int getNextDirection(int currentNode, int targetNode) {
  for (int direction = 0; direction < 3; direction++) {
    if (adjacencyList[currentNode][direction] == targetNode) {
      return direction; // Return the direction index
    }
  }
  return -1; // Invalid path
}

void navigatePath() {
  int currentNode = path[0]; // Start at the first node

  for (int i = 1; i < pathLength; i++) {
    int targetNode = path[i];
    int direction = getNextDirection(currentNode, targetNode);

    if (direction == -1) {
      Serial.print("No valid path from Node ");
      Serial.print(currentNode);
      Serial.print(" to Node ");
      Serial.println(targetNode);
      return; // Stop if no valid path
    }

    // Perform actions based on the direction
    switch (direction) {
      case 0: // reverse
        left(); left(); // 180-degree turn
        break;
      case 1: // Straight
        motor_drive(baseSpeed, baseSpeed);
        delay(500); // Adjust the delay based on distance
        break;
      case 2: // Left
        left();
        break;
      case 3:
        right();
        break;
    }

    // Update the current node
    currentNode = targetNode;

    // Add a short stop between movements for stability
    motor_drive(0, 0);
    delay(200);
  }

  // Stop the robot at the end of the path
  motor_drive(0, 0);
  Serial.println("Path completed.");
}

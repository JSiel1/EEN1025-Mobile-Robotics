#define TRIGGER_PIN_1 40
#define ECHO_PIN_1 41
#define TRIGGER_PIN_2 42
#define ECHO_PIN_2 45

#define OBSTACLE_DISTANCE 10 // Obstacle detection threshold in cm

void setup() {
    Serial.begin(115200);
    
    pinMode(TRIGGER_PIN_1, OUTPUT);
    pinMode(ECHO_PIN_1, INPUT);
    
    pinMode(TRIGGER_PIN_2, OUTPUT);
    pinMode(ECHO_PIN_2, INPUT);
}

float getDistance(int triggerPin, int echoPin) {
    long duration;
    float distance;

    // Trigger the sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Measure the time for echo
    duration = pulseIn(echoPin, HIGH, 30000); // Timeout at 30ms (~5m max distance)

    // Convert time to distance
    distance = duration * 0.0343 / 2;

    // If no valid measurement, return a large number
    if (distance <= 0 || distance > 400) {
        return 999; // Return a high value to indicate no detection
    }

    return distance;
}

bool detectObstacle() {
    float distance1 = getDistance(TRIGGER_PIN_1, ECHO_PIN_1);
    float distance2 = getDistance(TRIGGER_PIN_2, ECHO_PIN_2);

    Serial.print("Sensor 1 Distance: ");
    Serial.print(distance1);
    Serial.print(" cm | Sensor 2 Distance: ");
    Serial.print(distance2);
    Serial.println(" cm");

    // Check if either sensor detects an obstacle within the threshold
    if (distance1 <= OBSTACLE_DISTANCE || distance2 <= OBSTACLE_DISTANCE) {
        return true;
    }
    
    return false;
}

void loop() {
    if (detectObstacle()) {
        Serial.println("Obstacle detected! 🚨");
    } else {
        Serial.println("Path clear ✅");
    }

    delay(500); // Adjust delay based on required response time
}

#include <Pixy2.h>
#include <SPI.h>

// Define ESP32 pin connections for SPI communication with Pixy2
#define PIXY_MISO 19
#define PIXY_MOSI 23
#define PIXY_SCK  18
#define PIXY_SS   5

// Define ultrasonic sensor pins
#define ECHO_PIN 13
#define TRIG_PIN 14

// Define motor control pins
#define MOTOR_ENA 21
#define MOTOR_IN1 17
#define MOTOR_IN2 16
#define MOTOR_ENB 15
#define MOTOR_IN3 4
#define MOTOR_IN4 2

// Define maximum robot speed
#define MAX_SPEED 200

// Define object detection threshold distance
#define OBJECT_DISTANCE_THRESHOLD 30 // in cm

// Define tracking thresholds for x-coordinate
#define TRACKING_THRESHOLD_LEFT  150
#define TRACKING_THRESHOLD_RIGHT 170

Pixy2 pixy;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize Pixy2 camera
  SPI.begin();
  pixy.init();
  pixy.changeProg("color"); // Make sure Pixy2 is set to color tracking mode

  // Set motor control pins as outputs
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  // Set up ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

float getDistance() {
  // Trigger the ultrasonic sensor to measure distance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the time it takes for the ultrasonic pulse to return
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert the time to distance (in cm)
  float distance = duration * 0.034 / 2;

  return distance;
}

void moveRobot(int leftSpeed, int rightSpeed) {
  // Control motors
  analogWrite(MOTOR_ENA, abs(leftSpeed));
  analogWrite(MOTOR_ENB, abs(rightSpeed));

  // Set motor direction pins
  digitalWrite(MOTOR_IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, leftSpeed > 0 ? LOW : HIGH);
  digitalWrite(MOTOR_IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN4, rightSpeed > 0 ? LOW : HIGH);
}

void loop() {
  // Read object data from Pixy2 camera
  pixy.ccc.getBlocks(); // Use the correct method for color tracking

  // Check if blocks are detected
  if (pixy.ccc.numBlocks) {
    int x = pixy.ccc.blocks[0].m_x;
    int color = pixy.ccc.blocks[0].m_signature;

    // Calculate object distance using ultrasonic sensor
    float distance = getDistance();

    // Determine motor speeds based on object detection and distance
    int leftSpeed, rightSpeed;

    if (color == 1) { // Red object detected
      if (distance > OBJECT_DISTANCE_THRESHOLD) {
        // Adjust the tracking thresholds based on your setup
        if (x < TRACKING_THRESHOLD_LEFT) {
          // Object is on the left, turn left
          leftSpeed = -MAX_SPEED / 2;
          rightSpeed = MAX_SPEED;
        } else if (x > TRACKING_THRESHOLD_RIGHT) {
          // Object is on the right, turn right
          leftSpeed = MAX_SPEED;
          rightSpeed = -MAX_SPEED / 2;
        } else {
          // Object is in the center, move forward
          leftSpeed = MAX_SPEED;
          rightSpeed = MAX_SPEED;
        }
      } else {
        // Object is too close, stop or move backward
        leftSpeed = -MAX_SPEED / 2;
        rightSpeed = -MAX_SPEED / 2;
      }
    } else { // No red object detected
      // No object detected, stop the robot
      leftSpeed = 0;
      rightSpeed = 0;
    }

    // Move the robot based on tracking logic
    moveRobot(leftSpeed, rightSpeed);

    // Print sensor data to serial monitor
    Serial.print("Color: ");
    Serial.print(color);
    Serial.print(", Distance: ");
    Serial.print(distance);
    Serial.println();
  } else {
    // No blocks detected, stop the robot
    moveRobot(0, 0);
  }

  delay(50); // Add a small delay for stability
}

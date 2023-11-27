//dec50122 embedded robotic
//esp32+shield, l298n, maker line, pixy2, sr04, 4wheel chassis
//nov 2023
//bluetooth controlled mobile robot

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

//pin mapping
#define line_sensor 35  //maker line
#define miso  19  //pixy2
#define sck   18  
#define ss    5 
#define mosi  23
#define echo  13
#define trig  14  //dr04
#define ena   21
#define in1   17  //l298n
#define in2   16
#define in3   4
#define in4   2
#define enb   15  
#define encoder_right 32 //encoder
#define encoder_left  33  
#define laju  200 //robot speed
#define jarak_depan   15  //object infront in cm

// Define Bluetooth control commands
#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define STOP 'S'

// Add any other commands you need

void setup() {
  // Your existing setup code
  pinMode(line_sensor, INPUT); //line sensor analog
  pinMode(miso, OUTPUT);  //miso pixy2
  pinMode(sck, OUTPUT);//sck
  pinMode(ss, OUTPUT); //ss 
  pinMode(mosi, OUTPUT); //mosi
  pinMode(echo, INPUT); //echo sr04
  pinMode(trig, OUTPUT); //trig
  pinMode(ena, OUTPUT); //ena l298n
  pinMode(in1, OUTPUT); //in1
  pinMode(in2, OUTPUT); //in2
  pinMode(in3, OUTPUT); //in3
  pinMode(in4, OUTPUT); //in4
  pinMode(enb, OUTPUT); //enb
  pinMode(encoder_right, INPUT); //encoder m right
  pinMode(encoder_left, INPUT); //encoder m left
  //SerialBT.begin("YourRobotName", 115200);  // Set your Bluetooth device name
  SerialBT.begin("YourRobotName");  // Set your Bluetooth device name
}

void loop() {
  if (SerialBT.available()) { //check if bt communication is available
    char command = SerialBT.read(); //read bt com
    executeCommand(command);  //run command from smartphone.
  }
}

void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      moveForward();
      break;
    case BACKWARD:
      moveBackward();
      break;
    case LEFT:
      turnLeft();
      break;
    case RIGHT:
      turnRight();
      break;
    case STOP:
      stopMotors();
      break;
    // Add more cases for additional commands

    default:
      // Invalid command
      break;
  }
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ena, laju);  // Set the motor speed; 255 is full speed
  analogWrite(enb, laju);
}

// Function to move the robot backward
void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena, laju);  // Set the motor speed; 255 is full speed
  analogWrite(enb, laju);
}

// Function to stop the robot
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(ena, 0);  // Set the motor speed to 0 (stop)
  analogWrite(enb, 0);
}
// Function to turn the robot left
void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ena, laju);
  analogWrite(enb, laju);
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena, laju);
  analogWrite(enb, laju);
}

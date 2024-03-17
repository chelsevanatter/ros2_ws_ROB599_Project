#include <AccelStepper.h>
#include <Arduino.h>
#include "HX711.h"

// HX711 circuit wiring for load cell
const int LOADCELL_DOUT_PIN = 10;
const int LOADCELL_SCK_PIN = 11;

// Stepper motor objects
AccelStepper horizontal_stepper(AccelStepper::DRIVER, 6, 7); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper vertical_stepper(AccelStepper::DRIVER, 2, 3); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
// Enable horizontal = 8, 
// Enable vertical = 4

HX711 scale;

// Stretcher motion profile parameters (CHANGE THESE FOR DIFFERENT MOTIONS)
const int stretchSpeedMM = 50; // mm/s

// Stepper motor and leadscrew parameters (CHANGE THESE FOR HARDWARE CHANGES OR MICROSTEP CHANGES)
const float screwLead = 8; // mm/rotation
const float stepsPerRevolution = 200; // Takes into account microstepping

// Max force in grams that the test-rig should apply to a branch
const int maxForce = 1000;

// Distance in mm that gripper starts away from branch
const int distToBranch = 0; 

// Variables for keeping track of horizontal motor displacement
const float displacement_increment = 0.5;
float displacement = 0.5;


// Motion variables
float stretchSpeedSteps = 0;


bool setHeight = false;
bool startPushing = false;
bool goUp = false;
bool goDown = false;
bool goIn = false;
bool goOut = false;


// Variables for how long it pushes on the blueberry cane
const int numIterations = 90; // Change this to the desired number of iterations
float MMsToMove = 0.5; // Change this to set how far it goes on the first push
// float MMsIncrement = 0.5; // Change this to set how much further it goes on each subsequent push
int time_stamp = 0; // This is just the starting time_stamp
int t = 500; // This will be how long it's been since the first time_stamp but for now it just assumes each iteration takes 0.01 seconds


void setup() {
  Serial.begin(57600);

  // start the load cell and set the scale and tare it
  startLoadCell();

  // setup motor spped, acceleration, and current positions
  setupMotors();

}


void loop() {
  checkCommands();
  if (goDown == true) {
    myMoveHeight(-5);
    goDown = false;
    } 
  if (goUp == true){
    myMoveHeight(5);
    goUp = false;
  }
  if (goIn == true) {
    myMove(-5);
    goIn = false;
    } 
  if (goOut == true){
    myMove(5);
    goOut = false;
  }
  if (startPushing == true) {
    // This will push until the maximum force is reached or the maximum steps/iterations
    pushingLoop(); 
    startPushing = false;
  }
}


// Custom functions below



// Function for converting millimeters to steps
int mmToSteps(float distance) {
  int steps = round(distance * stepsPerRevolution / screwLead);
  return steps;
}
// Function for converting steps to millimeters
float stepsToMM(int steps) {
  int distance = steps / stepsPerRevolution * screwLead;
  return distance;
}

// Function for moving horizontal motor a set number of millimeters
void myMove(float MMs) {
  horizontal_stepper.move(-mmToSteps(MMs));
  // horizontal_stepper.setSpeed(mmToSteps(MMs));
  while (horizontal_stepper.distanceToGo() != 0) {
    horizontal_stepper.run();

  }

}
// Function for moving vertical motor a set number of millimeters
void myMoveHeight(float MMs) {
  vertical_stepper.move(mmToSteps(MMs));
  // vertical_stepper.setSpeed(mmToSteps(MMs));
  while (vertical_stepper.distanceToGo() != 0) {
    vertical_stepper.run();

  }

}

// Function for starting the load cell
void startLoadCell() {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
            
  scale.set_scale(433);        // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0
}

// Function for setting up the motors
void setupMotors() {

  // horizontal_stepper.setSpeed(stretchSpeedSteps); // Some function of hardware and desired stretch rate
  horizontal_stepper.setMaxSpeed(1000);
  horizontal_stepper.setAcceleration(15000);

  vertical_stepper.setMaxSpeed(1000); // Some function of hardware and desired stretch rate
  vertical_stepper.setAcceleration(15000);
  }

// Function for pushing on plant with horizontal motor
void pushingLoop() {
  // Read 3 values while the motor is not moving
  for (int j = 1; j < 3; j++){
    float loadCellReading = scale.get_units(); // Get load cell reading

    // Print data to serial (CSV format)
    Serial.print(j);
    Serial.print(", ");
    Serial.print(-1);
    Serial.print(", ");
    Serial.print(loadCellReading);
    Serial.print(", ");
    Serial.println(0);
  }
  
  // Move up to the branch
  myMove(distToBranch);

  float returnDistance = 0;
  
  for (int i = 1; i < numIterations; i++) {
    myMove(MMsToMove);
    float loadCellReading = scale.get_units(); // Get load cell reading

    // Print data to serial (CSV format)
    Serial.print(i);
    Serial.print(", ");
    Serial.print(time_stamp);
    Serial.print(", ");
    Serial.print(loadCellReading);
    Serial.print(", ");
    Serial.println(displacement);

    
    // MMsToMove = MMsToMove + MMsIncrement;
    
    displacement = displacement + displacement_increment;
    time_stamp = time_stamp+t;

    if (loadCellReading > maxForce){
      i = 100;
    }
    returnDistance += MMsToMove;

    delay(t);
  }

  myMove(-(returnDistance));
  }

  void checkCommands(){
    // Check if a command is available from Python
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the command from serial
    // Execute the command
    if (command == 'U'){
        goUp = true;
    } else if (command == 'D'){
        goDown = true;
    } else if (command == 'I'){
        goIn = true;
    } else if (command == 'O'){
        goOut = true;
    } else if (command == 'C') {
      startPushing = true;
    }
  }
  }
 

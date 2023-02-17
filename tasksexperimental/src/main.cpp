/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision18             vision        18              
// topLeft              motor         1               
// topRight             motor         10              
// bottomLeft           motor         11              
// bottomRight          motor         20              
// inertialSensor       inertial      4               
// Controller1          controller                    
// triggerMotor         motor         2               
// expansionMotor       motor         19              
// intakeMotor          motor         5               
// shootMotor           motor         6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
bool shooterOutward = true;
bool normalSpeed = true;
bool shootAuton = true;
bool isSkills = true;

//disregard these (SAS stands for StablilizedShotAuton), default values will be updated
double SSArpm = 200, SSAtime = 2;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//do this to clear the screen before you print to the brain
void readyToPrint() 
{
  Brain.Screen.setFont(mono40);
  Brain.Screen.clearLine(1, black);
  Brain.Screen.setCursor(Brain.Screen.row(), 1);
  Brain.Screen.setCursor(1, 1);
}

//clear the screen before printing to controller
void printToController() 
{
  Controller1.Screen.clearLine(2);
  Controller1.Screen.setCursor(2, 1);
}

// void printGuide(int mode)
// {
//   Controller1.Screen.clearLine(1);
//   Controller1.Screen.setCursor(1, 1);
//   if (mode == 0) //left
//   {
//     Controller1.Screen.print("<   ");
//   }
//   if (mode == 1) //right
//   {
//     Controller1.Screen.print(">   ");
//   }
//   if (mode == 2) //aligned
//   {
//     Controller1.Screen.print("GOOD");
//   }
//   if (mode == 3) //not there
//   {
//     Controller1.Screen.print("NONE");
//   }
// }

//NOTE: the pre auton calibration only works if you use the competition switch correctly
//1. before running program, put switch on autonomous, disable. 
//2. wait until screen says "done calibrating", and then switch to enable.
void pre_auton(void)
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) 
  {
    readyToPrint();
    Brain.Screen.print("calibrating!");
    wait(0.5, sec);
  }
  readyToPrint();
  Brain.Screen.print("done calibrating");
}

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

/////////////////////////////////////////////////////DRIVING STUFF
void stopDriving()
{
  topLeft.stop();
  topRight.stop();
  bottomLeft.stop();
  bottomRight.stop();
}

void driveTurnRight (int velocity, float turnDegrees)
{
  inertialSensor.setRotation(0, degrees);

  topLeft.setVelocity(velocity, percent);
  topRight.setVelocity(velocity, percent);
  bottomLeft.setVelocity(velocity, percent);
  bottomRight.setVelocity(velocity, percent);

  topLeft.spin(forward);
  topRight.spin(reverse);
  bottomLeft.spin(forward);
  bottomRight.spin(reverse);

  while (inertialSensor.rotation(degrees) < turnDegrees)
  {
    // Brain.Screen.print(inertialSensor.rotation(degrees)/turnDegrees);
    // readyToPrint();
  }
  stopDriving();
}

void driveTurnLeft (int velocity, float turnDegrees)
{
  inertialSensor.setRotation(0, degrees);

  topLeft.setVelocity(velocity, percent);
  topRight.setVelocity(velocity, percent);
  bottomLeft.setVelocity(velocity, percent);
  bottomRight.setVelocity(velocity, percent);

  topLeft.spin(reverse);
  topRight.spin(forward);
  bottomLeft.spin(reverse);
  bottomRight.spin(forward);

  while (inertialSensor.rotation(degrees) > -1*turnDegrees)
  {
    Brain.Screen.print(inertialSensor.rotation(degrees));
    readyToPrint();

  }
  stopDriving();
}

//********direction is -1 for backward and 1 for forward************
void strafe (int direction, int velocity, float cm) //doesnt work well in auton bc of gearing issues- it drifts diagonally
{
  float degreesNum = cm/0.08222; 

  topLeft.spinFor(forward, direction*degreesNum, degrees, false);
  topRight.spinFor(forward, direction*-1*degreesNum, degrees, false);
  bottomLeft.spinFor(forward, direction*-1*degreesNum, degrees, false);
  bottomRight.spinFor(forward, direction*degreesNum, degrees, true);

  topLeft.setVelocity(velocity, percent);
  topRight.setVelocity(velocity, percent);
  bottomLeft.setVelocity(velocity, percent);
  bottomRight.setVelocity(velocity, percent);
}

void forward_backward (int direction, int velocity, float cm)
{
  float degreesNum = cm/0.08866;
  topLeft.setVelocity(velocity, percent);
  topRight.setVelocity(velocity, percent);
  bottomLeft.setVelocity(velocity, percent);
  bottomRight.setVelocity(velocity, percent);

  topLeft.spinFor(forward, direction*degreesNum, degrees, false);
  topRight.spinFor(forward, direction*degreesNum, degrees, false);
  bottomLeft.spinFor(forward, direction*degreesNum, degrees, false);
  bottomRight.spinFor(forward, direction*degreesNum, degrees, false);

  Brain.Screen.print(degreesNum);
}

int intakeError; // sensor value - desired value : position
int intakePrevError = 0; //position 20 milliseconds ago
double fiftyDegrees = 50/0.08866;

int intakePID(float cm)
{
  topLeft.setPosition(0, degrees);
  bottomLeft.setPosition(0, degrees);
  topRight.setPosition(0, degrees);
  bottomRight.setPosition(0, degrees);

  int desiredDrive = cm/0.08866;
  bool over50 = false;
  if (abs(cm) > 50)
  {
    over50 = true;
  }

  topLeft.spin(forward);
  topRight.spin(forward);
  bottomLeft.spin(forward);
  bottomRight.spin(forward);
  intakeMotor.spin(forward);

  int coef = 0;
  if (cm >= 0)
  {
    coef = 1;
  }
  if (cm <= 0)
  {
    coef = -1;
  }

  int averagePosition = 0;
  int subtractFromPosition  = (abs(cm)-50)/0.08866;
  while (abs(averagePosition) <= abs(desiredDrive))
  {
    averagePosition = (topLeft.position(degrees) + bottomLeft.position(degrees) 
    + topRight.position(degrees) + bottomRight.position(degrees))/4;

    if (over50 == true)
    {
      if (abs(averagePosition)*0.08866 < 50)
      {
        intakeError = 100;
      }
      else
      {
        int degreesAfter50 = abs(averagePosition)-subtractFromPosition;
        intakeError = -85*(degreesAfter50-fiftyDegrees)/fiftyDegrees; //was 50
      }
    }
    else
    {
      intakeError = (averagePosition - desiredDrive)*-75/desiredDrive; //was 40
    }
    if(intakeError < 15) //usually 7
    {
      intakeError = 15;
    }

    readyToPrint();
    Brain.Screen.print(intakeError);
    
    intakeMotor.setVelocity(80, percent);
    topLeft.setVelocity(coef*intakeError, percent);
    topRight.setVelocity(coef*intakeError, percent);
    bottomLeft.setVelocity(coef*intakeError, percent);
    bottomRight.setVelocity(coef*intakeError, percent);

    intakePrevError = intakeError;
    vex::task::sleep(20);
  }
  stopDriving();
  intakeMotor.stop();
  return 1;
}

void triggerAuton(void)
{
  // triggerMotor.stop();
  triggerMotor.setVelocity(100, percent);
  triggerMotor.spin(forward);
  triggerMotor.spinFor(forward, 30, degrees, true);
  triggerMotor.spinFor(forward, -10, degrees, true);
  triggerMotor.stop();
}

////////////////////////////////////////////////////////////
//TURNING PID
////////////////////////////////////////////////////////////

bool enableTurnPID = true;
int turnError; // sensor value - desired value : position
int turnPrevError = 0; //position 20 milliseconds ago

int turnPID(float angle, int startSpeed)
{
  readyToPrint();
  Brain.Screen.print("starting turn");

  inertialSensor.setRotation(0, degrees);
  int desiredTurn = angle;

  topLeft.spin(forward);
  topRight.spin(forward);
  bottomLeft.spin(forward);
  bottomRight.spin(forward);

  int leftCoef = 0;
  int rightCoef = 0;
  if (angle >= 0)
  {
    rightCoef = -1;
    leftCoef = 1;
  }
    if (angle <= 0)
  {
    rightCoef = 1;
    leftCoef = -1;
  }
  while(enableTurnPID)
  {
    turnError = (inertialSensor.rotation(degrees) - desiredTurn)*-1*startSpeed/desiredTurn;

    // readyToPrint();
    // Brain.Screen.print(inertialSensor.rotation(degrees));

    if(turnError <1)
    {
      stopDriving();
      readyToPrint();
      Brain.Screen.print("done!");
      // wait(2, sec);
      break;
    }

    if (turnError < 10)
    {
      turnError = 10;
    }

    topLeft.setVelocity(leftCoef*turnError, percent);
    topRight.setVelocity(rightCoef*turnError, percent);
    bottomLeft.setVelocity(leftCoef*turnError, percent);
    bottomRight.setVelocity(rightCoef*turnError, percent);

    // prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }
  return 1;
}

////////////////////////////////////////////////////////////
//DRIVING PID
////////////////////////////////////////////////////////////
int driveError; // sensor value - desired value : position
int drivePrevError = 0; //position 20 milliseconds ago

int drivePID(int cm, int startVelocity)
{
  readyToPrint();
  // Brain.Screen.print("starting forward");

  topLeft.setPosition(0, degrees);
  bottomLeft.setPosition(0, degrees);
  topRight.setPosition(0, degrees);
  bottomRight.setPosition(0, degrees);

  int desiredDrive = cm/0.08866;
  bool over50 = false;
  if (abs(cm) > 50)
  {
    over50 = true;
  }

  topLeft.spin(forward);
  topRight.spin(forward);
  bottomLeft.spin(forward);
  bottomRight.spin(forward);

  int coef = 0;
  if (cm >= 0)
  {
    coef = 1;
  }
  if (cm <= 0)
  {
    coef = -1;
  }
  int averagePosition = 0;
  int subtractFromPosition  = (abs(cm)-50)/0.08866;
  while (abs(averagePosition) <= abs(desiredDrive))
  {
    averagePosition = (topLeft.position(degrees) + bottomLeft.position(degrees) 
    + topRight.position(degrees) + bottomRight.position(degrees))/4;

    if (over50 == true)
    {
      if (abs(averagePosition)*0.08866 < 50)
      {
        driveError = startVelocity;
      }
      else
      {
        // driveError = (abs(averagePosition)-fiftyDegrees)*-30/fiftyDegrees;
        int degreesAfter50 = abs(averagePosition)-subtractFromPosition;
        driveError = -1*startVelocity*(degreesAfter50-fiftyDegrees)/fiftyDegrees; //was 50
      }
    }
    else
    {
      driveError = (averagePosition - desiredDrive)*-1*startVelocity/desiredDrive; //was 40
    }

    if(driveError < 15) //usually 7
    {
      driveError = 15;
    }

    // readyToPrint();
    // Brain.Screen.print(driveError);
    
    topLeft.setVelocity(coef*driveError, percent);
    topRight.setVelocity(coef*driveError, percent);
    bottomLeft.setVelocity(coef*driveError, percent);
    bottomRight.setVelocity(coef*driveError, percent);

    drivePrevError = driveError;
    vex::task::sleep(20);
  }
  stopDriving();
  return 1;
}

void shootLow(int speed, int numDiscs) //shoot into the low goal
{
  wait(0.1, seconds);
  shootMotor.spin(forward);
  shootMotor.setVelocity(speed, percent);
  wait(1.5, seconds);

  for (int i = 1; i<= numDiscs; i++)
  {
    triggerMotor.spin(forward);
    triggerMotor.setVelocity(40, percent);
    wait(0.8, seconds);
    triggerMotor.stop();
    // wait(1, seconds);
  }
  shootMotor.stop();
}

vex::timer myTimer;
int stableSpinAuton()
{
  //SSArpm = 146;
  //SSAtime = 15;
  myTimer.clear();
  shootMotor.spin(forward);
  float autonVeloError;
  while(myTimer.time(seconds) < SSAtime)
  {
    autonVeloError = (SSArpm - shootMotor.velocity(rpm))/SSArpm;
    // shootMotor.setVelocity(0.62*200, rpm); //old/control w more oscillation
    readyToPrint();
    Brain.Screen.print(autonVeloError*100);
    shootMotor.setVelocity(shootMotor.velocity(percent) + 10*autonVeloError, percent); 
    //originally 10 can change
    wait(20, msec);
  }
  shootMotor.stop();
  return 0;
}

int stableSpinAuton(double setRpm, double setTime) {
  SSArpm = setRpm;
  SSAtime = setTime;
  task stableShootTask = task(stableSpinAuton);
  return 0;
}

void shootHigh (int speedRPM, int numDiscs) //shoot into the high goal
{
  wait(0.1, seconds);
  // shootMotor.spin(forward);
  // shootMotor.setVelocity(speedRPM, rpm);
  for (int i = 1; i<= numDiscs; i++)
  {
    wait(0.2, seconds);
    triggerMotor.spin(forward);
    triggerMotor.setVelocity(40, percent);
    wait(0.75, seconds);
    triggerMotor.stop();

    wait(0.5, seconds);
    intakeMotor.spin(forward);
    intakeMotor.setVelocity(80, percent);
    wait(0.5, seconds);
    intakeMotor.stop();

    wait(2.35, seconds); //was 2
  }
  // shootMotor.stop();
}

//never used (integrated manually)
void dipIntoRoller(bool firstRoller = false, bool goForDisk = false) {
  if (firstRoller) {
    drivePID(-5, 5);
  } else {
    drivePID(-55, 10);
  }
  wait(.1, seconds);
  intakeMotor.spin(forward);
  intakeMotor.setVelocity(60, percent);
  intakeMotor.spinFor(.4, seconds);
  if (firstRoller) {
    drivePID(5, 5);
    strafe(-1, 5, 10);
    if (goForDisk) {
      intakeMotor.setVelocity(60, percent);
      drivePID(40, 10);
      wait(1, seconds);
      intakeMotor.stop();
      return;
    }
  }
  drivePID(40, 10);
}

void AS4RollerExpansion(void) {
  int intendedNumDisks = 2;
  bool playingItSafe = true;
  bool takingShots = true;

  strafe(-1, 3, 15);
  turnPID(-5, 5); //<-- corrects strafe angle

  //dip into roller
  drivePID(-5, 5);
  wait(.1, seconds);
  intakeMotor.spin(forward);
  intakeMotor.setVelocity(65, percent);
  intakeMotor.spinFor(.45, seconds);
  /*if (intendedNumDisks > 2) {
    intakeMotor.setVelocity(60, percent);
    drivePID(40, 10);
    wait(1, seconds);
    intakeMotor.stop();
  }*/
  drivePID(50, 10);

  turnPID(94, 50);

  //dip into roller
  drivePID(-45, 10);
  wait(.1, seconds);
  intakeMotor.spin(forward);
  intakeMotor.setVelocity(65, percent);
  intakeMotor.spinFor(.45, seconds);
  if (takingShots) {
    stableSpinAuton(140, 5 + intendedNumDisks * 5);
  }
  drivePID(40, 10);

  //shooting
  wait(.5, seconds);
  if (takingShots) {
    turnPID(19, 5);
    wait(.125, seconds);
    shootHigh(140, intendedNumDisks);
    turnPID(-65, 5);
  } else {
    turnPID(-45, 5);
  }

  //either 2 rollers + expansion (true) or 4 rollers + expansion (false)
  if (!playingItSafe) {
    drivePID(250, 60);
    wait(.5, seconds);
    turnPID(-135, 50);

    //dip into roller
    dipIntoRoller();
    shootHigh(0.72*200, intendedNumDisks);
    turnPID(-90, 40);

    //dip into roller
    dipIntoRoller();

    wait(.5, seconds);
    turnPID(45, 40);
  }

  //drop expansion
  drivePID(10, 5);
  expansionMotor.spin(forward);
  expansionMotor.setVelocity(50, percent);
  expansionMotor.spinFor(.5, seconds);
  wait(2, seconds);

  drivePID(315, 90);
}

void pushLeft(void) //basic left auton that only does roller
{
  drivePID(-3, 5);
  intakeMotor.spin(forward);
  intakeMotor.setVelocity(40, percent);
  intakeMotor.spinFor(0.60, seconds); //roller, 0.85 best for comp
}

int senseRight(void) 
//this one is like the main right auton that shoots into the high goal
{
  while (inertialSensor.isCalibrating()) 
  {
    wait(0.1, seconds);
  }
  inertialSensor.setRotation(0, degrees);
  drivePID(68, 40);
  
  turnPID(-88, 80);
  drivePID(-7, 20);

  wait(0.25, seconds);
  intakeMotor.spin(forward);
  intakeMotor.setVelocity(20, percent);
  intakeMotor.spinFor(0.62, seconds); //roller, 0.85 best for comp
  
  wait(0.25, seconds);
  stopDriving();

  drivePID(2, 5);
  turnPID(20, 15); 
  wait(1.7, seconds); 
  shootHigh(0.76*200, 2);
  return 0;
}

void newAuton(void)
{
  task senseRightTask = task(senseRight);
  wait(1.5, seconds);
  stableSpinAuton(153, 15);
}

void nonSenseRight() //right auton that shoots into the low goal
{
  turnPID(90, 80);
  drivePID(62, 70);
  turnPID(-90, 80);
  drivePID(-7, 20);

  wait(0.25, seconds);

  intakeMotor.spin(forward);
  intakeMotor.setVelocity(20, percent);
  intakeMotor.spinFor(0.6, seconds); //roller, 0.85 best for comp

  stopDriving();

  drivePID(5, 60);
  turnPID(-80, 80);
  shootLow(50, 2);
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
// senseRight();
  if (isSkills) {
    AS4RollerExpansion();
  } else {
    newAuton();
  }
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void shootOut(void)
{
  shooterOutward = true;
  printToController();
  Controller1.Screen.print("SH_OUT");
}
void shootIn(void)
{
  shooterOutward = false;
  printToController();
  Controller1.Screen.print("SH_IN_");
}

void speedDown(void)
{
  normalSpeed = false;
  printToController();
  Controller1.Screen.print("57");
}
void speedUp(void)
{
  normalSpeed = true;
  printToController();
  Controller1.Screen.print("62");
}

void shoot(void)
{
  int coef = 1;
  if (shooterOutward)
  {
    coef = 1;
  }
  if (!shooterOutward)
  {
    coef = -1;
  }
  shootMotor.resetPosition();
  double veloError;
  while(Controller1.ButtonR2.pressing())
  {
    shootMotor.spin(forward);
    if (normalSpeed)
    {
      veloError = (124 - shootMotor.velocity(rpm))/124;
      // shootMotor.setVelocity(0.62*200, rpm); //old/control w more oscillation
      readyToPrint();
      Brain.Screen.print(veloError*100);
      shootMotor.setVelocity(shootMotor.velocity(percent) + 10*veloError, percent); 
      //originally 10 can change
    }
    if (!normalSpeed)
    {
      veloError = (114 - shootMotor.velocity(rpm))/114;
      // shootMotor.setVelocity(110, rpm); //old/control w more oscillation
      readyToPrint();
      Brain.Screen.print(veloError*100);
      shootMotor.setVelocity(shootMotor.velocity(percent) + 10*veloError, percent); 
      //originally 10 can change
    }
    // readyToPrint();
    // Brain.Screen.print("shoot");
    wait(20, msec);
  }
  // readyToPrint();
  // Brain.Screen.print("stop shoot");
  shootMotor.stop(brake);
}

void shootBack()
{
  while(Controller1.ButtonX.pressing())
  {
    shootMotor.spin(reverse);
    shootMotor.setVelocity(40, percent);
  }
shootMotor.stop();
}

void triggerBack(void)
{
  triggerMotor.setPosition(0, degrees);
  triggerMotor.spin(forward);
  triggerMotor.setVelocity(40, percent);
  while(Controller1.ButtonR1.pressing())
  {
    wait(20, msec);
  }
  triggerMotor.stop();
}

void triggerForward(void)
{
  triggerMotor.setPosition(0, degrees);
  triggerMotor.spin(forward);
  triggerMotor.setVelocity(-40, percent);
  while(Controller1.ButtonB.pressing())
  {
    wait(20, msec);
  }
  triggerMotor.stop();
}

void intakeForward(void)
{
  while(Controller1.ButtonL1.pressing())
  {
    intakeMotor.spin(forward);
    intakeMotor.setVelocity(100, percent);
    readyToPrint();
    Brain.Screen.print("intake forward");
    wait(20, msec);
  }
  readyToPrint();
  Brain.Screen.print("stop intake forward");
  intakeMotor.stop();
}

void intakeBackward(void)
{
  while(Controller1.ButtonL2.pressing())
  {
    intakeMotor.spin(forward);
    intakeMotor.setVelocity(-100, percent);
    readyToPrint();
    Brain.Screen.print("intake back");
    wait(20, msec);
  }
  readyToPrint();
  Brain.Screen.print("stop intake back");
  intakeMotor.stop();
}

void expandForward(void)
{
    readyToPrint();
    Brain.Screen.print("expand forward");
    while(Controller1.ButtonA.pressing())
    {
    expansionMotor.spin(forward);
    expansionMotor.setVelocity(40, percent);
    wait(20, msec);
    }
    expansionMotor.stop();
  }

void usercontrol(void) {
  // User control code here, inside the loop
  // shootMotor.setVelocity(30, percent);
  // shootMotor.spin(forward);
  Controller1.Screen.clearScreen(); 
  enableTurnPID = false;
  topLeft.spin(forward);
  topRight.spin(forward);
  bottomLeft.spin(forward);
  bottomRight.spin(forward);
  while (1) {
    //this set will make all of the moving directions work together
    //note that this might make the velocity bigger than 100 but 
    //im not sure if that will cause any problems

    Controller1.ButtonR2.pressed(shoot);
    Controller1.ButtonR1.pressed(triggerBack);
    Controller1.ButtonB.pressed(triggerForward);

    Controller1.ButtonL1.pressed(intakeForward);
    Controller1.ButtonL2.pressed(intakeBackward);

    Controller1.ButtonA.pressed(expandForward);
    
    // Controller1.ButtonY.pressed(shootOut);
    // Controller1.ButtonA.pressed(shootIn);

    Controller1.ButtonUp.pressed(speedUp);
    Controller1.ButtonDown.pressed(speedDown);
    Controller1.ButtonX.pressed(shootBack);



    int topLeftV;
    int topRightV;
    int bottomLeftV;
    int bottomRightV;
    topLeftV = Controller1.Axis4.value() + Controller1.Axis3.value() + 
    0.5*Controller1.Axis1.value();
    topRightV = -1*Controller1.Axis4.value() + Controller1.Axis3.value() + 
    -0.5*Controller1.Axis1.value();
    bottomLeftV = -1*Controller1.Axis4.value() + Controller1.Axis3.value() + 
    0.5*Controller1.Axis1.value();
    bottomRightV = Controller1.Axis4.value() + Controller1.Axis3.value() + 
    -0.5*Controller1.Axis1.value();

    topLeft.setVelocity(topLeftV, percent);
    topRight.setVelocity(topRightV, percent);
    bottomLeft.setVelocity(bottomLeftV, percent);
    bottomRight.setVelocity(bottomRightV, percent);                                                                                                                                                                                                                                                                                                                                                                                   

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
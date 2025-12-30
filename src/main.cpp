/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ximin                                                     */
/*    Created:      12/26/2025, 3:10:04 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
brain Brain;

// A global instance of competition
competition Competition;

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

// Robot configuration code.
// Robot configuration code.
// Robot configuration code.
motor Left12 = motor(PORT12, ratio6_1, true);
motor Left16 = motor(PORT16, ratio6_1, true);
motor Left17 = motor(PORT17, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(Left12, Left16, Left17);
motor Right3 = motor(PORT3, ratio6_1, false);
motor Right4 = motor(PORT4, ratio6_1, false);
motor Right5 = motor(PORT5, ratio6_1, false);
motor_group RightDriveSmart = motor_group(Right3, Right4, Right5);
inertial DrivetrainInertial = inertial(PORT6);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 320, 40, mm, 0.6666666666666666);

motor Intake = motor(PORT9, ratio6_1, false);

motor Arm = motor(PORT10, ratio36_1, true);

controller Controller1 = controller(primary);

digital_out match_load = digital_out(Brain.ThreeWirePort.A);
digital_out change_height = digital_out(Brain.ThreeWirePort.B);
digital_out descore = digital_out(Brain.ThreeWirePort.C);

// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}

bool vexcode_initial_drivetrain_calibration_completed = false;
void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  vexcode_initial_drivetrain_calibration_completed = true;
  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

void vexcodeInit() {

  // Calibrate the Drivetrain
  calibrateDrivetrain();

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  bool matchLoadState = false;
  bool changeHeightState = false;
  bool descoreState = false;

  bool aPressedLast = false;
  bool bPressedLast = false;
  bool cPressedLast = false;

  bool pressed = false;
  bool first_time = true;
  bool driveStopping = false;
  timer driveStopTimer;
  timer l1Timer;
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // stop the motors if the brain is calibrating
      if (DrivetrainInertial.isCalibrating()) {
        LeftDriveSmart.stop();
        RightDriveSmart.stop();
        while (DrivetrainInertial.isCalibrating()) {
          wait(25, msec);
        }
      }
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int left = Controller1.Axis3.position() + Controller1.Axis1.position();
      int right = Controller1.Axis3.position() - Controller1.Axis1.position();

      // Deadband
      if (abs(left) < 5) left = 0;
      if (abs(right) < 5) right = 0;

      if (left == 0 && right == 0) {

          // first frame of stopping
          if (!driveStopping) {
              driveStopping = true;
              driveStopTimer.reset();

              LeftDriveSmart.stop(brake);
              RightDriveSmart.stop(brake);
          }

          // after 0.5s → coast
          else if (driveStopTimer.time(sec) > 0.5) {
              LeftDriveSmart.stop(coast);
              RightDriveSmart.stop(coast);
          }

      }
      else {
          // driver moved again → cancel stopping logic
          driveStopping = false;

          LeftDriveSmart.setVelocity(left, percent);
          RightDriveSmart.setVelocity(right, percent);
          LeftDriveSmart.spin(forward);
          RightDriveSmart.spin(forward);
      }

      // ================= ARM SMART HOLD =================
      double armRPM = Arm.velocity(rpm); 

      if(armRPM < -3){
        Brain.Screen.print("Hello World!");
      }
      else{
        Brain.Screen.clearScreen();
      }

      // L1 – up control (50% if B toggled, else 100%)
      if (Controller1.ButtonL1.pressing()) {
        int upSpeed = changeHeightState ? 25 : 100;
        Arm.spin(forward, upSpeed, pct);
        first_time = true;
        pressed = true;
        Controller1LeftShoulderControlMotorsStopped = false;
      }

      // L2 – manual down (full power)
      else if (Controller1.ButtonL2.pressing()) {
        Arm.spin(reverse, 100, pct);
        Controller1LeftShoulderControlMotorsStopped = false;
      }

      // Auto hold
      else {
        if (pressed) { 
            if (first_time) {
                l1Timer.reset();  // start the 0.5s timer
                first_time = false;
            }
            if (l1Timer.time(sec) < 0.75) {
                // Reverse kick
                if (l1Timer.time(sec) < 0.3 ){
                  Arm.spin(reverse, 40, pct);
                  Controller1LeftShoulderControlMotorsStopped = false;
                }
                else if (armRPM < -3){
                  Arm.spin(reverse, 40, pct);
                  Controller1LeftShoulderControlMotorsStopped = false;
                }
                else{
                  // 0.5s passed → go back to auto-hold
                  pressed = false;   // reset pressed so this block won't run again
                  first_time = true; // ready for next L1 press
                }
            } else {
                // 0.5s passed → go back to auto-hold
                pressed = false;   // reset pressed so this block won't run again
                first_time = true; // ready for next L1 press
            }
        } 

        if (!pressed) {
            // Normal auto-hold
              Arm.stop(coast);
              Controller1LeftShoulderControlMotorsStopped = true;
            
        }
    }




      // check the ButtonR1/ButtonR2 status to control Intake
      if (Controller1.ButtonR1.pressing()) {
        Intake.spin(forward, 100, pct);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Intake.spin(reverse, 100, pct);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Intake.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
            bool aPressedNow = Controller1.ButtonA.pressing();
      if (aPressedNow && !aPressedLast) {
        matchLoadState = !matchLoadState;
        match_load.set(matchLoadState);
      }
      aPressedLast = aPressedNow;

      // ---- CHANGE HEIGHT (Button B) ----
      bool bPressedNow = Controller1.ButtonB.pressing();
      if (bPressedNow && !bPressedLast) {
        changeHeightState = !changeHeightState;
        change_height.set(changeHeightState);
      }
      bPressedLast = bPressedNow;

      // ---- DESCORE (Button C / X) ----
      bool cPressedNow = Controller1.ButtonX.pressing();   // (C = X on V5 controller)
      if (cPressedNow && !cPressedLast) {
        descoreState = !descoreState;
        descore.set(descoreState);
      }
      cPressedLast = cPressedNow;
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Begin project code

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  // place automonous code here
}

void userControl(void) {
  Brain.Screen.clearScreen();
  // place driver control in this while loop
  while (true) {
    wait(20, msec);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Arm.resetPosition();
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
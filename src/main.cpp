/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\User                                             */
/*    Created:      Wed Mar 27 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include "joystick.cpp"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

enum gameElements {
  mobileGoal,
  redRing,
  blueRing,
};
using gameElements = enum gameElements;
competition  Competition;

motor leftFrontDriveMotor = motor(PORT17, ratio18_1, false);
motor leftBackDriveMotor = motor(PORT19, ratio18_1, false);//no longer exists
motor rightFrontDriveMotor = motor(PORT7, ratio18_1, false);//no longer exists
motor rightBackDriveMotor = motor(PORT8, ratio18_1, false);
motor frontDriveMotor = motor(PORT9, ratio18_1, false);
motor backDriveMotor = motor(PORT20, ratio18_1, false);
inertial inertialSens = inertial(PORT4);
//motor deploy = motor(PORT2, ratio18_1, false);
//motor climbMotorA = motor(PORT6, ratio18_1, false);
//motor climbMotorB = motor(PORT16, ratio36_1, true);
//motor_group climb = motor_group(climbMotorA, climbMotorB);
motor_group leftDrive = motor_group(leftFrontDriveMotor, leftBackDriveMotor);//still used in drive functions and i already wrote the code dont want to go in and change all my trackging wheels and relabel everything. this is a later me problem
motor_group rightDrive = motor_group(rightFrontDriveMotor, rightBackDriveMotor);//still used in drive functions and i already wrote the code dont want to go in and change all my trackging wheels and relabel everything. this is a later me problem
motor mogo = motor(PORT12, ratio36_1, false);
motor upperarm = motor(PORT11, ratio6_1, false);
motor lowerarm = motor(PORT5, ratio18_1, true);
motor plunger = motor(PORT13, ratio18_1, false);
//vision vision15 = vision(PORT15);
distance distanceSensor = distance(PORT14);
aivision::colordesc AIVision15__Red(1, 214, 66, 112, 35, 0.44);
aivision::colordesc AIVision15__Blue(2, 63, 168, 198, 30, 0.68);
aivision::colordesc AIVision15__Mogo(3, 126, 161, 87, 31, 0.57);
vex::aivision AIVision15(PORT15, AIVision15__Red, AIVision15__Blue, AIVision15__Mogo, aivision::ALL_AIOBJS);
//using the climb group to run the new arm group to have two different motor carts run in tandem
controller con1 = controller(primary);


class Drives {
  public:
    void static robotOriented(double xIn, double yIn, double turnIn) {
      double leftMotor = yIn + turnIn;
      double rightMotor = -yIn + turnIn;
      double frontMotor = xIn + turnIn;
      double backMotor = -xIn + turnIn;

      leftDrive.spin(fwd, leftMotor, velocityUnits::pct);
      rightDrive.spin(fwd, rightMotor, velocityUnits::pct);
      frontDriveMotor.spin(fwd, frontMotor, velocityUnits::pct);
      backDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }



    void static fieldOriented() {
      double headingRadians = inertialSens.heading() * 3.141592/180;
      double yInput = getLeftY(con1, 5);
      double xInput = getLeftX(con1, 5);
      double sineHeading = sin(headingRadians);
      double cosHeading = cos(headingRadians);
      double rotatedYInput = xInput * sineHeading + yInput * cosHeading;
      double rotatedXInput = xInput * cosHeading - yInput * sineHeading;
      double turning = getTrueRightX(con1, 5);
      double leftMotor = rotatedYInput + turning;
      double rightMotor = -rotatedYInput + turning;
      double FrontMotor = rotatedXInput + turning;
      double backMotor = -rotatedXInput + turning;

      leftDrive.spin(fwd, leftMotor, velocityUnits::pct);
      rightDrive.spin(fwd, rightMotor, velocityUnits::pct);
      frontDriveMotor.spin(fwd, FrontMotor, velocityUnits::pct);
      backDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }
    
    static void turnToHeading(double targetHeading, double maxSpeed) {
      double headingCurrent = inertialSens.heading(degrees);
      double error = targetHeading - headingCurrent;

      // Normalize error to the range [-180, 180]
      if (error > 180) {
          error -= 360;
      } else if (error < -180) {
          error += 360;
      }

      // Determine turn direction and speed
      double turnSpeed = maxSpeed * (error > 0 ? 1 : -1);

      // Turn the robot until it reaches the target heading
      while (std::abs(error) > 1) { // 1-degree tolerance
          headingCurrent = inertialSens.heading(degrees);
          error = targetHeading - headingCurrent;

          // Normalize error again
          if (error > 180) {
              error -= 360;
          } else if (error < -180) {
              error += 360;
          }

          // Adjust speed as the robot gets closer to the target
          if (std::abs(error) < 30) {
              turnSpeed = (error > 0 ? 10 : -10); // Slow down near the target
          }

          // Turn the robot
          Drives::robotOriented(0.0, 0.0, turnSpeed);
          task::sleep(10); // Small delay to prevent overloading the CPU
      }

      // Stop the robot once the target heading is reached
      Drives::robotOriented(0.0, 0.0, 0.0);
  }
};

void usercontrol(void) {
  int tuningvariable = 0;
  int mogotuningvariable = 0;
  int startpos = 0;
  int upperstartpos = 0 + tuningvariable;
  int mogopos = 1550 + tuningvariable;
  int wallpos = 3100 + tuningvariable;
  int intakepos = -650 + tuningvariable;
  int pushpos = -800;
  int retractpos = 0;
  int mogotracker = 0;
  int plungertracker = 0;
  int lowerwallpos = 160;
  int mogointakepos = -280  + mogotuningvariable;
  int mogoholdpos = -70 + mogotuningvariable;
  int toggletracker = 0;
  double headingcurrent = 0.0;
  upperarm.setTimeout(2 , seconds);
  lowerarm.setTimeout(2 , seconds);
  mogo.setTimeout(2 , seconds);
  plunger.setTimeout(2 , seconds);
  con1.Screen.clearScreen();
  con1.Screen.setCursor(1, 1);
  upperarm.setStopping(hold);
  double ringcheck = 0.0;
  double ringturncheck = 0.0;
  double ringdifference = 0.0;
double objdistance = 0.0;
  //con1.Screen.print("toggle tracker: %d", toggletracker);
  while(true)
  {
    AIVision15.takeSnapshot(AIVision15__Mogo);
con1.Screen.clearScreen();
con1.Screen.setCursor(2, 1);
    con1.Screen.print("object at: (%d, %d)", AIVision15.largestObject.centerX, AIVision15.largestObject.centerY);
    con1.Screen.setCursor(3, 1);
    objdistance = distanceSensor.objectDistance(inches);
    con1.Screen.print("distance: %f", objdistance);

    Drives::fieldOriented();
    //zeroes inertial sensor
    


    if(mogo.position(degrees) < (-180)){
      mogotracker = 1;
    }
    else if(mogo.position(degrees) > (-180)){
      mogotracker = 0;
    }
    if(plunger.position(degrees) < (-300)){
      plungertracker = 1;
    }
    else if(plunger.position(degrees) > (-100)){
      plungertracker = 0;
    }
    if(con1.ButtonY.pressing()) {
      if(toggletracker==0) {
        if (Brain.Timer.time(msec)>400.0) {
         toggletracker = 1;
         con1.rumble("-");
          con1.Screen.clearScreen();
          con1.Screen.setCursor(1, 1);
          con1.Screen.print("toggle tracker: %d", toggletracker);
        Brain.Timer.clear();
        }
      }
      else if(toggletracker==1) {
        if (Brain.Timer.time(msec)>400.0) {
          toggletracker = 0;
          con1.rumble("-");
          con1.Screen.clearScreen();
          con1.Screen.setCursor(1, 1);
          con1.Screen.print("toggle tracker: %d", toggletracker);
          Brain.Timer.clear();
         }
      }
  
    }

    if (con1.ButtonA.pressing()) {
      if (plungertracker == 0) {
        plunger.spinToPosition(pushpos, degrees, false);
        plunger.setStopping(hold);
      } else if (plungertracker == 1) {
        plunger.spinToPosition(retractpos, degrees, false);
        plunger.setStopping(hold);
      } else {
        con1.Screen.print("Error: plunger tracker not set to 0 or 1");
      }
   
    }
    
    if (toggletracker==0) {

if(con1.ButtonX.pressing()) {

ringcheck = distanceSensor.objectDistance(inches);
if (ringcheck < 8.0)
{
if (ringcheck <0.1)
{
  Drives::robotOriented(0.0, 0.0, 0.0);
  con1.rumble(". .");
}

else if (ringcheck > 0.5) {
  
Drives::robotOriented(0.0, 0.0, 10.0);
wait(50, msec);
ringturncheck = distanceSensor.objectDistance(inches);
if(ringcheck - ringturncheck > 0.0) {
ringcheck = ringcheck - ringturncheck;
ringdifference = ringcheck - ringturncheck;
  while(ringcheck >= ringdifference)
  {
    ringcheck = ringdifference;
    wait(50, msec);
    ringdifference = ringturncheck - distanceSensor.objectDistance(inches);
  }
  Drives::robotOriented(0.0, 0.0, 0.0);
  Drives::robotOriented(0.0, 10.0, 0.0);
  waitUntil(distanceSensor.objectDistance(inches) <= 0.5);
  con1.rumble("_");
}
else {
  ringcheck = distanceSensor.objectDistance(inches);
  Drives::robotOriented(0.0, 0.0, -10.0);
  wait(50, msec);
  ringturncheck = distanceSensor.objectDistance(inches);
  if(ringcheck - ringturncheck > 0.0) {
  ringcheck = ringcheck - ringturncheck;
  ringdifference = ringcheck - ringturncheck;
    while(ringcheck >= ringdifference)
    {
      ringcheck = ringdifference;
      wait(50, msec);
      ringdifference = ringturncheck - distanceSensor.objectDistance(inches);
    }
    Drives::robotOriented(0.0, 0.0, 0.0);
    Drives::robotOriented(0.0, 10.0, 0.0);
    waitUntil(distanceSensor.objectDistance(inches) <= 0.5);
  con1.rumble("_");
}
else {Drives::robotOriented(0.0, 0.0, 0.0);}
}
}
}
}

if(con1.ButtonB.pressing()) {
  mogo.setPosition(0, degrees);
  con1.Screen.clearScreen();
  con1.Screen.setCursor(1, 1);
  con1.Screen.print("upperarm position: %d", upperarm.position(degrees));
}

      if (con1.ButtonL1.pressing()) {
      if(mogotracker == 0) {
        mogo.spinToPosition(mogointakepos, degrees, false);
        mogo.setStopping(hold);
       }
      else if(mogotracker == 1) {
        mogo.spinToPosition(mogoholdpos, degrees, false);
        mogo.setStopping(hold);
       }
        else{
          con1.Screen.print("Error: mogo tracker not set to 0 or 1");
       }
      }
       if (con1.ButtonL2.pressing()) {

        AIVision15.takeSnapshot(AIVision15__Mogo);
        if(AIVision15.objectCount > 0 && distanceSensor.objectDistance(inches) < 24) {
          // Object detected
          int centerX = AIVision15.largestObject.centerX;

          if (centerX < 165) {
              // Object is on the left
              Drives::robotOriented(0.0, 0.0, -10.0); 
              while(AIVision15.largestObject.centerX <= 145 && distanceSensor.objectDistance(inches) < 24 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
              while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 24 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              // Turn left
              con1.rumble("_");
          }
        
          else if (centerX > 175) {
              // Object is on the right
              Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
              while(AIVision15.largestObject.centerX >= 195 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)

              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
              while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              con1.rumble("_");
          }
          
          else if(centerX >= 165 && centerX <= 175) {
            Drives::robotOriented(0.0, 0.0, 0.0);
            con1.rumble(". .");
             // Stop
            if (distanceSensor.objectDistance(inches) < 5.0) {
              Drives::robotOriented(0.0, -15.0, 0.0);
              waitUntil(distanceSensor.objectDistance(inches) >= 5.0);
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
               // Stop
               
               if (centerX < 165) {
                // Object is on the left
                Drives::robotOriented(0.0, 0.0, -10.0); 
                while(AIVision15.largestObject.centerX <= 150 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
                while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                // Turn left
                con1.rumble("_ _");
            }
          
            else if (centerX > 175) {
                // Object is on the right
                Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
                while(AIVision15.largestObject.centerX >= 195 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
  
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
                while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                
            }
            con1.rumble("_ _");
            }
            else if (distanceSensor.objectDistance(inches) > 5.1) {
              Drives::robotOriented(0.0, 15.0, 0.0);
              waitUntil(distanceSensor.objectDistance(inches) <= 5.1);
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
               // Stop

               if (centerX < 165) {
                // Object is on the left
                Drives::robotOriented(0.0, 0.0, -10.0); 
                while(AIVision15.largestObject.centerX <= 150 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
                while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                // Turn left

            }
          
            else if (centerX > 175) {
                // Object is on the right
                Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
                while(AIVision15.largestObject.centerX >= 190 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
  
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
                while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop

            }
            con1.rumble("_ _");
            }
          }
          }

      task::sleep(20); // Small delay to prevent CPU overload
  }

      
      if (con1.ButtonR1.pressing()){
        if(upperarm.position(degrees) < (-100)) {
          upperarm.spinToPosition(upperstartpos, degrees, false);        
       }
        else if(upperarm.position(degrees) < (20)) {
          upperarm.spinToPosition(mogopos, degrees, false);
       }
        else if(upperarm.position(degrees) > (1500)) {
            upperarm.spinToPosition(wallpos, degrees, false);
            lowerarm.spinToPosition(lowerwallpos, degrees, false);
       }
      }
      if (con1.ButtonR2.pressing()){
        if(upperarm.position(degrees) > (1900)) {
          upperarm.spinToPosition(mogopos, degrees, false); 
          lowerarm.spinToPosition(startpos, degrees, false);       
       }
        else if(upperarm.position(degrees) > (1500)) {
          upperarm.spinToPosition(startpos, degrees, false);
       }
        else if(upperarm.position(degrees) > (-25)){
            upperarm.spinToPosition(intakepos, degrees, false);
       } 
     }
     if(con1.ButtonLeft.pressing()) {
Drives::turnToHeading(270.0, 100.0);
      }
      if(con1.ButtonRight.pressing()) {

        Drives::turnToHeading(90.0, 100.0);
      }
      if(con1.ButtonUp.pressing()) {
 
        Drives::turnToHeading(0.0, 100.0);
      }
      if(con1.ButtonDown.pressing()) {
        headingcurrent = inertialSens.heading(degrees);
        if(180-headingcurrent < 0) {
          Drives::robotOriented(0.0, 0.0, -100.0);
          waitUntil(inertialSens.heading(degrees) <= 215);
          Drives::robotOriented(0.0, 0.0, -30.0);
          waitUntil(inertialSens.heading(degrees) <= 200);
          Drives::robotOriented(0.0, 0.0, -10.0);
          waitUntil(inertialSens.heading(degrees) <= 181);
          Drives::robotOriented(0.0, 0.0, 0.0);
        } else {
          Drives::robotOriented(0.0, 0.0, 100.0);
          waitUntil(inertialSens.heading(degrees) >= 145);
          Drives::robotOriented(0.0, 0.0, 30.0);
          waitUntil(inertialSens.heading(degrees) >= 160);
          Drives::robotOriented(0.0, 0.0, 10.0);
          waitUntil(inertialSens.heading(degrees) >= 179);
          Drives::robotOriented(0.0, 0.0, 0.0);
        }
      }
    }
    if (toggletracker==1) {

      if(con1.ButtonX.pressing()) {
        inertialSens.setHeading(0.0, degrees);
        inertialSens.setRotation(0.0, degrees);
        inertialSens.startCalibration();
        con1.rumble(".");
        while (inertialSens.isCalibrating()) {
            task::sleep(10);
        }
        inertialSens.setHeading(0.0, degrees);
        inertialSens.setRotation(0.0, degrees);
      }

      if(con1.ButtonB.pressing()) {
        upperarm.setPosition(0, degrees);
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print("upperarmreset");
      }
    
      if (con1.ButtonL1.pressing()) {
      mogo.spin(forward);
     } 
    else if (con1.ButtonL2.pressing()) {
      mogo.spin(reverse);
     } 
    else  {
      mogo.stop();
     }
    if (con1.ButtonR1.pressing()) {
      upperarm.spin(forward);
      upperarm.setStopping(brake);
     } 
     else if (con1.ButtonR2.pressing()) {
      upperarm.spin(reverse);
      upperarm.setStopping(brake);
     } 
     else  {
      upperarm.stop();
     }
     if(con1.ButtonLeft.pressing()) {
      mogotuningvariable = mogotuningvariable - 10;
      con1.Screen.clearScreen();
  con1.Screen.setCursor(1, 1);
  con1.Screen.print("mogo tuning: %d", mogotuningvariable);
  con1.Screen.setCursor(2, 1);
  con1.Screen.print("mogointakepos: %d", mogointakepos);
  con1.Screen.setCursor(3, 1);
  con1.Screen.print("mogoposition: %d", mogo.position(degrees));
     }
    if(con1.ButtonRight.pressing()) {
      mogotuningvariable = mogotuningvariable + 10;
      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print("mogo tuning: %d", mogotuningvariable);
      con1.Screen.setCursor(2, 1);
      con1.Screen.print("mogointakepos: %d", mogointakepos);
      con1.Screen.setCursor(3, 1);
      con1.Screen.print("mogoposition: %d", mogo.position(degrees));
    }
    if(con1.ButtonUp.pressing()) {
      tuningvariable = tuningvariable + 10;
      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print("tuning variable: %d", tuningvariable);
      con1.Screen.setCursor(2, 1);
      con1.Screen.print("intakepos: %d", intakepos);
      con1.Screen.setCursor(3, 1);
      con1.Screen.print("upperarmposition: %d", upperarm.position(degrees));
    }
    if(con1.ButtonDown.pressing()) {
      tuningvariable = tuningvariable - 10;
      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print("tuning variable: %d", tuningvariable);
      con1.Screen.setCursor(2, 1);
      con1.Screen.print("intakepos: %d", intakepos);
      con1.Screen.setCursor(3, 1);
      con1.Screen.print("upperarmposition: %d", upperarm.position(degrees));
    }
    
    }
  }
}



void auton(void) {

  int tuningvariable = 0;
  int upperstartpos = 50 + tuningvariable;
  int mogopos = 1600 + tuningvariable;
  int intakepos = -650 + tuningvariable;
  int pushpos = -800;
  int retractpos = 0;
  int intakeposabsolute = std::abs(intakepos);


  int autonchoice = 7;


  upperarm.setStopping(hold);
                                         
if(autonchoice == 1) {
  //Redsoloautonslot1 

  inertialSens.setHeading(270.0, degrees);
  inertialSens.setRotation(270.0, degrees);
  inertialSens.startCalibration();
  while(inertialSens.isCalibrating()) {
    task::sleep(10);
  }
  inertialSens.setHeading(270.0, degrees);
  inertialSens.setRotation(270.0, degrees);


upperarm.spinToPosition(upperstartpos + 50, degrees, false);



Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning

//first drive direction X axis code
frontDriveMotor.setPosition(0, degrees);//sets the front drive motor to 0 degrees
Drives::robotOriented(50.0, 0.0, 0.0);//sets motors to drive in negitive x based on robot
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.0));//waits until the front drive motor hits the degrees of rotation to hit 30inches
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.25))//if the front drive motor has rotate enought to have moved 30.25 inches or more
{
  Drives::robotOriented(-15.0, 0.0, 0.0);//turns the robot right to correct for overshoot
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(34.1));////waits until the front drive motor is at 30.1 or less inches of travel based on rotation
}
Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning
//first drive direction Y axis code same as x but with y
Drives::turnToHeading(270.0, 30);
Drives::turnToHeading(270.0, 10.0);
upperarm.setTimeout(2, seconds);
upperarm.spinToPosition(intakepos-50, degrees, true);
wait(500, msec);

Drives::robotOriented(0.0, 0.0, 0.0);
task::sleep(20);
upperarm.spinToPosition(mogopos, degrees, false);

Drives::turnToHeading(270.0, 25.0);
Drives::turnToHeading(270.0, 10.0);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(-30.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(30.0, 0.0, 0.0);
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 40.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.5));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.75))
 {
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(17.6));
}
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, true);
upperarm.spinToPosition(mogopos+200, degrees, true);
upperarm.spinToPosition(mogopos-100, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(pushpos, degrees, true);
upperarm.spinToPosition(mogopos+100, degrees, true);
upperarm.spinToPosition(mogopos-50, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);


frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(-50.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(28.0));
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(28.25))
{
  Drives::robotOriented(15.0, 0.0, 0.0);
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(28.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);


upperarm.spinToPosition(mogopos-900, degrees, true);
leftFrontDriveMotor.setPosition(0, degrees);
Drives::turnToHeading(270.0, 20.0);
Drives::robotOriented(0.0, 50.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(21.0));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(21.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(21.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);



upperarm.spinToPosition(upperstartpos-200, degrees, true);

upperarm.spinToPosition(mogopos + 100, degrees, true);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(7.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);
Drives::turnToHeading(180.0, 40.0);
  Drives::turnToHeading(180.0, 15.0);
Drives::robotOriented(0.0, 0.0, 0.0);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, -30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(30.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);


}
    
  


if(autonchoice == 2) {

  //bluesoloautonslot2


inertialSens.setHeading(90.0, degrees);
  inertialSens.setRotation(90.0, degrees);
  inertialSens.startCalibration();
  while(inertialSens.isCalibrating()) {
    task::sleep(10);
  }
  inertialSens.setHeading(90.0, degrees);
  inertialSens.setRotation(90.0, degrees);


upperarm.spinToPosition(upperstartpos + 50, degrees, false);



Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning

//first drive direction X axis code
frontDriveMotor.setPosition(0, degrees);//sets the front drive motor to 0 degrees
Drives::robotOriented(-50.0, 0.0, 0.0);//sets motors to drive in negitive x based on robot
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.0));//waits until the front drive motor hits the degrees of rotation to hit 30inches
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.25))//if the front drive motor has rotate enought to have moved 30.25 inches or more
{
  Drives::robotOriented(15.0, 0.0, 0.0);//turns the robot right to correct for overshoot
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(34.1));////waits until the front drive motor is at 30.1 or less inches of travel based on rotation
}
Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning
//first drive direction Y axis code same as x but with y
Drives::turnToHeading(90.0, 30);
Drives::turnToHeading(90.0, 10.0);
upperarm.setTimeout(2, seconds);
upperarm.spinToPosition(intakepos-50, degrees, true);
wait(500, msec);

Drives::robotOriented(0.0, 0.0, 0.0);
task::sleep(20);
upperarm.spinToPosition(mogopos, degrees, false);

Drives::turnToHeading(90.0, 25.0);
Drives::turnToHeading(90.0, 10.0);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(30.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(-30.0, 0.0, 0.0);
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 40.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.5));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.75))
 {
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(17.6));
}
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, true);
upperarm.spinToPosition(mogopos+200, degrees, true);
upperarm.spinToPosition(mogopos-100, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(pushpos, degrees, true);
upperarm.spinToPosition(mogopos+100, degrees, true);
upperarm.spinToPosition(mogopos-50, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);


frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(50.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(28.0));
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(28.25))
{
  Drives::robotOriented(-15.0, 0.0, 0.0);
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(28.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);


upperarm.spinToPosition(mogopos-900, degrees, true);
leftFrontDriveMotor.setPosition(0, degrees);
Drives::turnToHeading(90.0, 20.0);
Drives::robotOriented(0.0, 50.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(21.0));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(21.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(21.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);



upperarm.spinToPosition(upperstartpos-200, degrees, true);

upperarm.spinToPosition(mogopos + 100, degrees, true);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(7.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);
Drives::turnToHeading(180.0, 40.0);
  Drives::turnToHeading(180.0, 15.0);
Drives::robotOriented(0.0, 0.0, 0.0);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, -30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(30.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);


}

if(autonchoice == 3) {

//redleftautonslot3


upperarm.spinToPosition(mogopos, degrees, true);

leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 50.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches) <= 5.1);
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, true);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(30.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(24.0));
Drives::robotOriented(0.0, 0.0, 0.0);

}

if(autonchoice == 4) {
  //bluerightautonslot4
  
upperarm.spinToPosition(mogopos, degrees, true);

leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 50.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches) <= 5.1);
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, true);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(-30.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(24.0));

Drives::robotOriented(0.0, 0.0, 0.0);
}

  if(autonchoice == 5) {
    //redleftautonslot5

  //set subsystems at a standardized height pre-match set up slot 8
  

  upperarm.setMaxTorque(70, pct);
  upperarm.setVelocity(30, pct);
  upperarm.setStopping(coast);
  upperarm.spin(reverse);
  task::sleep(3000);
  upperarm.stop();
  upperarm.setPosition(0, degrees);
  upperarm.setMaxTorque(100, pct);
upperarm.spinToPosition(intakeposabsolute, degrees, true);
inertialSens.setHeading(0.0, degrees);
  inertialSens.setRotation(0.0, degrees);
  upperarm.setPosition(0, degrees);
  inertialSens.startCalibration();
  while(inertialSens.isCalibrating()) {
    task::sleep(10);
  }
  inertialSens.setHeading(0.0, degrees);
  inertialSens.setRotation(0.0, degrees);
  }


  if(autonchoice == 6) {

    //Risky faster red right slot 6
    //redrightautonslot6


  inertialSens.setHeading(270.0, degrees);
  inertialSens.setRotation(270.0, degrees);

task::sleep(20);
  upperarm.setPosition(0, degrees);
upperarm.spinToPosition(upperstartpos + 50, degrees, false);



Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning

//first drive direction X axis code
frontDriveMotor.setPosition(0, degrees);//sets the front drive motor to 0 degrees
Drives::robotOriented(60.0, 0.0, 0.0);//sets motors to drive in negitive x based on robot
/*waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.0));//waits until the front drive motor hits the degrees of rotation to hit 30inches
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.25))//if the front drive motor has rotate enought to have moved 30.25 inches or more
{
  Drives::robotOriented(-15.0, 0.0, 0.0);//turns the robot right to correct for overshoot
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(34.1));////waits until the front drive motor is at 30.1 or less inches of travel based on rotation
}
  */
  waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(20.0));
  waitUntil(distanceSensor.objectDistance(inches) <= 0.8);
Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning
//first drive direction Y axis code same as x but with y
Drives::turnToHeading(270.0, 30); 
Drives::turnToHeading(270.0, 10.0);
upperarm.setTimeout(2, seconds);
upperarm.spinToPosition(intakepos-50, degrees, true);
wait(500, msec);

Drives::robotOriented(0.0, 0.0, 0.0);
task::sleep(20);
upperarm.spinToPosition(mogopos, degrees, false);

Drives::turnToHeading(270.0, 25.0);
Drives::turnToHeading(270.0, 10.0);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(-50.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(30.0, 0.0, 0.0);
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 80.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.5));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.75))
 {
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(17.6));
}
Drives::robotOriented(0.0, 0.0, 0.0);

//test alignment



AIVision15.takeSnapshot(AIVision15__Mogo);
        if(AIVision15.objectCount > 0 && distanceSensor.objectDistance(inches) < 24) {
          // Object detected
          int centerX = AIVision15.largestObject.centerX;

          if (centerX < 165) {
              // Object is on the left
              Drives::robotOriented(0.0, 0.0, -10.0); 
              while(AIVision15.largestObject.centerX <= 145 && distanceSensor.objectDistance(inches) < 24 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
              while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 24 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              // Turn left
              con1.rumble("_");
          }
        
          else if (centerX > 175) {
              // Object is on the right
              Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
              while(AIVision15.largestObject.centerX >= 195 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)

              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
              while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              con1.rumble("_");
          }
          
          else if(centerX >= 165 && centerX <= 175) {
            Drives::robotOriented(0.0, 0.0, 0.0);
            con1.rumble(". .");
             // Stop
            if (distanceSensor.objectDistance(inches) < 5.0) {
              Drives::robotOriented(0.0, -15.0, 0.0);
              waitUntil(distanceSensor.objectDistance(inches) >= 5.0);
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
               // Stop
               
               if (centerX < 165) {
                // Object is on the left
                Drives::robotOriented(0.0, 0.0, -10.0); 
                while(AIVision15.largestObject.centerX <= 150 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
                while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                // Turn left
                con1.rumble("_ _");
            }
          
            else if (centerX > 175) {
                // Object is on the right
                Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
                while(AIVision15.largestObject.centerX >= 195 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
  
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
                while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                
            }
            con1.rumble("_ _");
          }
          else if (distanceSensor.objectDistance(inches) > 5.1) {
            Drives::robotOriented(0.0, 15.0, 0.0);
            waitUntil(distanceSensor.objectDistance(inches) <= 5.1);
            Drives::robotOriented(0.0, 0.0, 0.0); // Stop
             // Stop

             if (centerX < 165) {
              // Object is on the left
              Drives::robotOriented(0.0, 0.0, -10.0); 
              while(AIVision15.largestObject.centerX <= 150 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
              while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              // Turn left

          }
        
          else if (centerX > 175) {
              // Object is on the right
              Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
              while(AIVision15.largestObject.centerX >= 190 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)

              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
              while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop

          }
          con1.rumble("_ _");
          }
        }
      }
//end of test alignment


plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, true);
upperarm.spinToPosition(mogopos+200, degrees, true);
upperarm.spinToPosition(mogopos-100, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(pushpos, degrees, true);
upperarm.spinToPosition(mogopos+100, degrees, true);
upperarm.spinToPosition(mogopos-50, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);

Drives::turnToHeading(270.0, 20.0);
Drives::turnToHeading(270.0, 10.0);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(-70.0, 0.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches)>10);
waitUntil(distanceSensor.objectDistance(inches)<30);
double pos1 = frontDriveMotor.position(degrees);
Drives::robotOriented(-30.0, 0.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches)>30);
Drives::robotOriented(0.0, 0.0, 0.0);
double pos2 = frontDriveMotor.position(degrees);
double pos1point5 = (pos1 + pos2) / 2   ;
Drives::robotOriented(30.0, 0.0, 0.0);
waitUntil(frontDriveMotor.position(degrees) == pos1point5);
Drives::robotOriented(0.0, 0.0, 0.0);
upperarm.spinToPosition(mogopos-900, degrees, true);
leftFrontDriveMotor.setPosition(0, degrees);
Drives::turnToHeading(270.0, 20.0);
Drives::robotOriented(0.0, 80.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches)< 0.8);
Drives::robotOriented(0.0, 0.0, 0.0);



upperarm.spinToPosition(upperstartpos-200, degrees, true);

upperarm.spinToPosition(mogopos + 100, degrees, true);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 70.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(7.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);
Drives::turnToHeading(180.0, 40.0);
  Drives::turnToHeading(180.0, 15.0);
Drives::robotOriented(0.0, 0.0, 0.0);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 70.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, -80.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(20));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(20.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(20.1));
}
Drives::robotOriented(0.0, -30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(30.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

}

if(autonchoice == 7) {
  //riskybluerleftautonslot7
 
  inertialSens.setHeading(90.0, degrees);
  inertialSens.setRotation(90.0, degrees);

  task::sleep(20);
  upperarm.setPosition(0, degrees);
upperarm.spinToPosition(upperstartpos + 50, degrees, false);



Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning

//first drive direction X axis code
frontDriveMotor.setPosition(0, degrees);//sets the front drive motor to 0 degrees
Drives::robotOriented(-60.0, 0.0, 0.0);//sets motors to drive in negitive x based on robot
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(12.0));
Drives::robotOriented(-50.0, 0.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches) <= 2.0|| std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.0));
/*waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.0));//waits until the front drive motor hits the degrees of rotation to hit 30inches
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(34.25))//if the front drive motor has rotate enought to have moved 30.25 inches or more
{
  Drives::robotOriented(15.0, 0.0, 0.0);//turns the robot right to correct for overshoot
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(34.1));////waits until the front drive motor is at 30.1 or less inches of travel based on rotation
}*/
Drives::robotOriented(15.0, 0.0, 0.0);
wait(500, msec);//stops the robot from turning
Drives::robotOriented(0.0, 0.0, 0.0);//stops the robot from turning
//first drive direction Y axis code same as x but with y
Drives::turnToHeading(90.0, 30);
Drives::turnToHeading(90.0, 10.0);
upperarm.setTimeout(2, seconds);
upperarm.spinToPosition(intakepos-50, degrees, true);
wait(200, msec);

Drives::robotOriented(0.0, 0.0, 0.0);
task::sleep(20);
upperarm.spinToPosition(mogopos, degrees, false);

Drives::turnToHeading(90.0, 25.0);
Drives::turnToHeading(90.0, 10.0);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(30.0, 0.0, 0.0);
waitUntil(std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(frontDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(-50.0, 0.0, 0.0);
  waitUntil(std::abs(frontDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 80.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.5));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(17.75))
 {
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(17.6));
}
Drives::robotOriented(0.0, 0.0, 0.0);
//test alignment



AIVision15.takeSnapshot(AIVision15__Mogo);
        if(AIVision15.objectCount > 0 && distanceSensor.objectDistance(inches) < 24) {
          // Object detected
          int centerX = AIVision15.largestObject.centerX;

          if (centerX < 165) {
              // Object is on the left
              Drives::robotOriented(0.0, 0.0, -10.0); 
              while(AIVision15.largestObject.centerX <= 145 && distanceSensor.objectDistance(inches) < 24 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
              while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 24 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              // Turn left
              con1.rumble("_");
          }
        
          else if (centerX > 175) {
              // Object is on the right
              Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
              while(AIVision15.largestObject.centerX >= 195 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)

              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
              while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              con1.rumble("_");
          }
          
          else if(centerX >= 165 && centerX <= 175) {
            Drives::robotOriented(0.0, 0.0, 0.0);
            con1.rumble(". .");
             // Stop
            if (distanceSensor.objectDistance(inches) < 5.0) {
              Drives::robotOriented(0.0, -15.0, 0.0);
              waitUntil(distanceSensor.objectDistance(inches) >= 5.0);
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
               // Stop
               
               if (centerX < 165) {
                // Object is on the left
                Drives::robotOriented(0.0, 0.0, -10.0); 
                while(AIVision15.largestObject.centerX <= 150 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
                while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                // Turn left
                con1.rumble("_ _");
            }
          
            else if (centerX > 175) {
                // Object is on the right
                Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
                while(AIVision15.largestObject.centerX >= 195 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
  
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
                while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
                {
                  AIVision15.takeSnapshot(AIVision15__Mogo);
                }
                Drives::robotOriented(0.0, 0.0, 0.0); // Stop
                
            }
            con1.rumble("_ _");
          }
          else if (distanceSensor.objectDistance(inches) > 5.1) {
            Drives::robotOriented(0.0, 15.0, 0.0);
            waitUntil(distanceSensor.objectDistance(inches) <= 5.1);
            Drives::robotOriented(0.0, 0.0, 0.0); // Stop
             // Stop

             if (centerX < 165) {
              // Object is on the left
              Drives::robotOriented(0.0, 0.0, -10.0); 
              while(AIVision15.largestObject.centerX <= 150 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, -5.0); // Turn left
              while(AIVision15.largestObject.centerX <= 160 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop
              // Turn left

          }
        
          else if (centerX > 175) {
              // Object is on the right
              Drives::robotOriented(0.0, 0.0, 10.0); // Turn right
              while(AIVision15.largestObject.centerX >= 190 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)

              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 5.0); // Turn right
              while(AIVision15.largestObject.centerX >= 180 && distanceSensor.objectDistance(inches) < 22 && AIVision15.objectCount > 0)
              {
                AIVision15.takeSnapshot(AIVision15__Mogo);
              }
              Drives::robotOriented(0.0, 0.0, 0.0); // Stop

          }
          con1.rumble("_ _");
          }
        }
      }
//end of test alignment
plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, true);
upperarm.spinToPosition(mogopos+200, degrees, true);
upperarm.spinToPosition(mogopos-100, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(pushpos, degrees, true);
upperarm.spinToPosition(mogopos+100, degrees, true);
upperarm.spinToPosition(mogopos-50, degrees, true);
upperarm.spinToPosition(mogopos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);

Drives::turnToHeading(90.0, 20.0);
Drives::turnToHeading(90.0, 10.0);

frontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(70.0, 0.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches)>10);
wait(500, msec);
waitUntil(distanceSensor.objectDistance(inches)<36);
Drives::robotOriented(0.0, 0.0, 0.0);
upperarm.spinToPosition(mogopos-900, degrees, true);
leftFrontDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 80.0, 0.0);
waitUntil(distanceSensor.objectDistance(inches)< 0.8);
Drives::robotOriented(0.0, 0.0, 0.0);


upperarm.spinToPosition(mogopos-900, degrees, true);
leftFrontDriveMotor.setPosition(0, degrees);
Drives::turnToHeading(90.0, 20.0);
Drives::robotOriented(0.0, 80.0, 0.0);
waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(21.0));
if (std::abs(leftFrontDriveMotor.position(degrees)) >= calRotation(21.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(leftFrontDriveMotor.position(degrees)) <= calRotation(21.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);



upperarm.spinToPosition(upperstartpos-200, degrees, true);

upperarm.spinToPosition(mogopos + 100, degrees, true);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 70.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(7.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(7.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);
Drives::turnToHeading(180.0, 40.0);
  Drives::turnToHeading(180.0, 15.0);
Drives::robotOriented(0.0, 0.0, 0.0);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, 70.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.0));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(2.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(2.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);

plunger.spinToPosition(pushpos, degrees, true);
plunger.spinToPosition(retractpos, degrees, false);

rightBackDriveMotor.setPosition(0, degrees);
Drives::robotOriented(0.0, -80.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(20));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(20.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(20.1));
}
Drives::robotOriented(0.0, -30.0, 0.0);
waitUntil(std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30));
if (std::abs(rightBackDriveMotor.position(degrees)) >= calRotation(30.25))
{
  Drives::robotOriented(0.0, -15.0, 0.0);
  waitUntil(std::abs(rightBackDriveMotor.position(degrees)) <= calRotation(30.1));
}
Drives::robotOriented(0.0, 0.0, 0.0);
}


}


int main() {
  upperarm.setPosition(0, degrees);
  lowerarm.setPosition(0, degrees);
  plunger.setPosition(0, degrees);
  mogo.setPosition(0, degrees);
  mogo.setMaxTorque(100, pct);
  mogo.setVelocity(70, pct);
  upperarm.setMaxTorque(100, pct);
  upperarm.setVelocity(100, pct);
  lowerarm.setMaxTorque(100, pct);
  lowerarm.setVelocity(50, pct);
  plunger.setMaxTorque(50, pct);
  plunger.setVelocity(80, pct);
  lowerarm.setStopping(brake);
  upperarm.setStopping(hold);
  plunger.setStopping(brake);
  mogo.setStopping(hold);
  //climbMotorA.setVelocity(100, rpm);
  //climbMotorB.setVelocity(100, rpm);
 // climbMotorA.setMaxTorque(100, pct);
 // climbMotorB.setMaxTorque(100, pct);
 // climb.setStopping(hold);

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  inertialSens.setHeading(0.0, degrees);
  inertialSens.setRotation(0.0, degrees);
  inertialSens.startCalibration();
  while(inertialSens.isCalibrating()) {
    task::sleep(10);
  }
  inertialSens.setHeading(0.0, degrees);
  inertialSens.setRotation(0.0, degrees);

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);
   
  while(true) {
    task::sleep(10);
  }

}

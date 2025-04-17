#include "vex.h"
#include <cmath>



/* 
  this class was made by Cole Huffine, class of 2024
  lets hope rivera gives this to future classes bc
  i think it would be useful for everyone
*/

/*
  In order to use this class, put it in your "include" folder
  and then put '#inlcude "joystick.cpp"' at the top of your main file

  Usage:
    If you need to get the value of a joystick fixed to account for the physical limitations
      getLeftX(con, deadband);
      getLeftY(con, deadband);
      getRightX(con, deadband);
      getRightY(con, deadband);

    If you need to get the REAL value of a joystick, as inputted into the brain
      getTrueLeftX(con, deadband);
      getTrueLeftY(con, deadband);
      getTrueRightX(con, deadband);
      getTrueRightY(con, deadband);
*/


//this returns the sign of the value you pass in as a parameter
//i originally made this file in java, and c++ doesnt have this
//built in so i made it :)
int signum(double val) {
  return (val >= 0) ? 1 : -1;
}

//use these if you need to get the true value of any joystick
double getTrueLeftX(vex::controller con, int deadband) {
  if (abs(con.Axis4.position(pct)) > deadband) {
    return con.Axis4.position(pct);
  } else {
    return 0;
  }
}

double getTrueLeftY(vex::controller con, int deadband) {
  if (abs(con.Axis3.position(pct)) > deadband) {
    return con.Axis3.position(pct);
  } else {
    return 0;
  }
}

double getTrueRightX(vex::controller con, int deadband) {
  if (abs(con.Axis1.position(pct)) > deadband) {
    return con.Axis1.position(pct);
  } else {
    return 0;
  }
}

double getTrueRightY(vex::controller con, int deadband) {
 if (abs(con.Axis2.position(pct)) > deadband) {
    return con.Axis2.position(pct);
  } else {
    return 0;
  }
}

/*
  * This method is to fix the flawed input of a vex controller. The input is a box,
  * both X and Y axis can go from -1 to 1. However, since the controller build physically
  * forces the joystick into a circle, its impossible to reach the the corner of the box.
  * This method fixes that issue.
*/  
double joystickFix(bool isLeft, bool isX, vex::controller con) {
  double root2 = sqrt(2);
  double x = isLeft ? getTrueLeftX(con, 0) : getTrueRightX(con, 0);
  double y = isLeft ? getTrueLeftY(con, 0) : getTrueRightY(con, 0);
  double magnitude = sqrt(x*x + y*y);
  double values[2] = {
    signum(x) * fmin(fabs(x*root2), magnitude),
    signum(y) * fmin(fabs(y*root2), magnitude)
  };
  return isX? values[0] : values[1];
}


// these return the fixed value of all of the joysticks
double getLeftX(vex::controller con, int deadband) {
  if(fabs(joystickFix(true, true, con)) > deadband){
    return joystickFix(true, true, con);
  } else {
    return 0;
  }
}

double getLeftY(vex::controller con, int deadband) {
  if(fabs(joystickFix(true, false, con)) > deadband){
    return joystickFix(true, false, con);
  } else {
    return 0;
  }
}

double getRightX(vex::controller con, int deadband) {
  if(fabs(joystickFix(false, true, con)) > deadband){
    return joystickFix(false, true, con);
  } else {
    return 0;
  }
}

double getRightY(vex::controller con, int deadband) {
  if(fabs(joystickFix(false, false, con)) > deadband){
    return joystickFix(false, false, con);
  } else {
    return 0;
  }
}

// Function to calculate degrees of wheel rotation for a given distance
double calRotation(double distanceInInches) {
  // Diameter of the wheel in inches
  const double wheelDiameter = 3.25;
  // Circumference of the wheel (C = π * d)
  const double wheelCircumference = M_PI * wheelDiameter;
  // Gear ratio (1 rotation of the motor = 360 degrees)
  const double degreesPerRotation = 360.0;

  // Calculate the number of rotations needed to cover the distance
  double rotations = distanceInInches / wheelCircumference;

  // Convert rotations to degrees
  double degrees = rotations * degreesPerRotation;

  return degrees;
}


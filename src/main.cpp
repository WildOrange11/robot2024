#include "vex.h"
using namespace vex;

// define your global instances of motors and other devices here
brain B;

motor chain = motor(PORT1, ratio18_1);
motor LMotor1 = motor(PORT2, true); // For these motors - Add "reverse" if necessary
motor LMotor2 = motor(PORT3, true);
motor LMotor3 = motor(PORT4, true);
motor RMotor1 = motor(PORT5);
motor RMotor2 = motor(PORT6);
motor RMotor3 = motor(PORT7);
motor intake = motor(PORT9);

motor_group L1 = motor_group(LMotor1, LMotor2, LMotor3);
motor_group R1 = motor_group(RMotor1, RMotor2, RMotor3);

drivetrain d1 = drivetrain(L1, R1, 260, 400, 390);

digital_out goal1 = digital_out(B.ThreeWirePort.A);

controller C1;

int fbpos = 0;
int lrpos = 0;
int armpos = 0;
int f1 = 800;
int f2 = 6;

competition Competition;

void PickRing()
{
  chain.spin(forward);
  intake.spin(forward);
}

void PickGoal()
{
  goal1.set(false);
}

void DropRing()
{
  goal1.set(false);
  intake.spinFor(1000, deg);
  chain.spinFor(-1000, deg);
}

void PlaceGoal()
{
  goal1.set(true);
}

void pre_auton(void)
{
  chain.setVelocity(100, percent);
  intake.setVelocity(100, percent);
  L1.setVelocity(100, percent);
  R1.setVelocity(100, percent);
  intake.spin(forward);
}

void autonomous(void)
{
  // There are 4 possible starting positions of the robot. The robot will always be placed so that the center of the robot is on the starting line...

  // Code for red alliance, right side
  chain.spinFor(forward, 1000, deg, false);
  d1.driveFor(18, inches);
  d1.turnFor(90, deg);
}


  void usercontrol(void)
{
  while (1)
  {
    // Get the controller positions
    fbpos = C1.Axis3.position();  // Forward/Backward
    lrpos = C1.Axis1.position();  // Left/Right

    // Apply controller input to drivetrain
    L1.spin(forward, fbpos - lrpos, percent);  // Left motors
    R1.spin(forward, fbpos + lrpos, percent);  // Right motors

    // Chain control with ButtonL2
    if (C1.ButtonL2.pressing())
    {
      chain.spin(forward);
    }
    else
    {
      chain.stop();
    }

    // Intake control with ButtonR1
    if (C1.ButtonR1.pressing())
    {
      intake.spin(forward);
    }
    else
    {
      intake.stop();
    }

    // Goal control with ButtonR2
    if (C1.ButtonR2.pressing())
    {
      goal1.set(false);
    }
    else
    {
      goal1.set(true);
    }

    wait(20, msec);  // Small delay to prevent max CPU load
  }
}

int main()
{
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true)
  {
    wait(0.5, sec);
  }
}
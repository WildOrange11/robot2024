#include "vex.h"

using namespace vex;

// define your global instances of motors and other devices here
brain B;

motor LMotor1 = motor(PORT1, ratio6_1);
motor LMotor2 = motor(PORT2, ratio6_1);
motor LMotor3 = motor(PORT3, ratio6_1);
motor RMotor1 = motor(PORT4, ratio6_1);
motor RMotor2 = motor(PORT5, ratio6_1);
motor RMotor3 = motor(PORT6, ratio6_1);
motor chain = motor(PORT7,ratio18_1);

motor_group L1 = motor_group(LMotor1,LMotor2,LMotor3);
motor_group R1 = motor_group(RMotor1,RMotor2,RMotor3);

controller C1;

int fbpos = 0;
int lrpos = 0;
int armpos = 0;
int inpos = 0;
int d1 = 800;
int d2 = 6;



competition Competition;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void setSpeed(int lv, int rv){
  LMotor1.setVelocity(lv,percent);
  LMotor2.setVelocity(lv,percent);
  LMotor3.setVelocity(lv,percent);
  RMotor1.setVelocity(rv,percent);
  RMotor2.setVelocity(rv,percent);
  RMotor3.setVelocity(rv,percent);
}

void PlaceRing(void){
  goal1.set(true);
  chain.spinFor(1000,deg);
  goal1.set(false);
  wait(0.5,sec);
}

void pre_auton(void) {
  chain.setVelocity(100,percent);
  setSpeed(80,80);
  LMotor1.setPosition(0,deg);
  LMotor2.setPosition(0,deg);
  LMotor3.setPosition(0,deg);
  RMotor1.setPosition(0,deg);
  RMotor2.setPosition(0,deg);
  RMotor3.setPosition(0,deg);
  chain.setPosition(0,deg);
}

void autonomous(void) {

  setSpeed(80,80);
  //Pick up the first ring
  LMotor1.spinFor(d1,deg,false);
  LMotor2.spinFor(d1,deg,false);
  LMotor3.spinFor(d1,deg,false);
  RMotor1.spinFor(d1,deg,false);
  RMotor2.spinFor(d1,deg,false);
  RMotor3.spinFor(d1,deg);
  PlaceRing();
  LMotor1.spinFor(-d1,deg,false);
  LMotor2.spinFor(-d1,deg,false);
  LMotor3.spinFor(-d1,deg,false);
  RMotor1.spinFor(-d1,deg,false);
  RMotor2.spinFor(-d1,deg,false);
  RMotor3.spinFor(-d1,deg);
  wait(0.5,sec);
  
}


void usercontrol(void){
  while (1) {
    fbpos = C1.Axis2.position();
    lrpos = C1.Axis1.position();
    armpos = C1.Axis3.position();
    if(fbpos > 10){
      LMotor1.spin(forward);
      LMotor2.spin(forward);
      LMotor3.spin(forward);
      RMotor1.spin(forward);
      RMotor2.spin(forward);
      RMotor3.spin(forward);
    }else if(fbpos < -10){
      LMotor1.spin(reverse);
      LMotor2.spin(reverse);
      LMotor3.spin(reverse);
      RMotor1.spin(reverse);
      RMotor2.spin(reverse);
      RMotor3.spin(reverse);
    }else{
      LMotor1.stop();
      LMotor2.stop();
      LMotor3.stop();
      RMotor1.stop();
      RMotor2.stop();
      RMotor3.stop();
    }
    
    if(lrpos != 0){
      setSpeed(50-lrpos/2.1,50+lrpos/2.1);
      LMotor1.spin(forward);
      LMotor2.spin(forward);
      LMotor3.spin(forward);
      RMotor1.spin(forward);
      RMotor2.spin(forward);
      RMotor3.spin(forward);
    }else{
      setSpeed(80,80);
}

    if(armpos > 10){
      chain.spin(forward);
    }else if(armpos < -10){
      chain.spin(reverse);
    }else{
      chain.stop();
      }
    if(C1.ButtonR1.pressing()){
      chain.spin(forward);
    }else{
      chain.stop();
    }if(C1.ButtonR2.pressing())
      {goal1.set(true)      }
      if(C1.ButtonL2.pressing()){
        goal1.set(false)
      }

  
}}

int main() {
 
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}


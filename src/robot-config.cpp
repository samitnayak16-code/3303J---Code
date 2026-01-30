#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller1;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);
motor Left1= motor(PORT12, ratio6_1, true);
motor Left2 = motor(PORT13, ratio6_1, true);
motor Left3 = motor(PORT11, ratio6_1, true);
motor Right1 = motor(PORT18, ratio6_1, false);
motor Right2 = motor(PORT16, ratio6_1, false);
motor Right3 = motor(PORT17, ratio6_1, false);
motor Intake1 = motor(PORT15, ratio6_1, false);
motor Intake2 = motor(PORT1, ratio6_1, true);

//Add your devices below, and don't forget to do the same in robot-config.h:


optical ColorSortSensor = optical(PORT9);
inertial IMU = inertial(PORT14);

digital_out CenterGoal = digital_out(Brain.ThreeWirePort.D);
digital_out MatchLoad = digital_out(Brain.ThreeWirePort.B);
digital_out Descore = digital_out(Brain.ThreeWirePort.C);
digital_out Blocker = digital_out(Brain.ThreeWirePort.E);

void vexcodeInit( void ) {
  // nothing to initialize
}
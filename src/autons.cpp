#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 7, 0, 27, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}


/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

bool flagMatchLoad1 = false;
bool flagMatchLoad2 = false;
bool flagMatchLoad3 = false;
bool flagMatchLoad4 = false;
bool flagIntakeJam = false;
bool Autocontrol = true;

void matchloadTask() 
{
  while(Autocontrol)
  {
    if(flagMatchLoad1)
    {
      wait(1100,msec);           // wait 1000 milliseconds
      MatchLoad.set(true);        // Activate piston
      wait(200,msec);  
      MatchLoad.set(false);
      flagMatchLoad1 = false;
    }
    if(flagMatchLoad2)
    {
      wait(700,msec);           // wait 800 milliseconds
      MatchLoad.set(true);        // Activate piston
      flagMatchLoad2 = false;
    }
    if(flagMatchLoad3)
    {
      wait(300,msec);           
      MatchLoad.set(true);        // Activate piston
      wait(300,msec);  
      MatchLoad.set(false); 
      flagMatchLoad3 = false;
    }
    if(flagMatchLoad4)
    {
      wait(600,msec);           // wait 800 milliseconds
      MatchLoad.set(true);        // Activate piston
      flagMatchLoad2 = false;
    }
    if(flagIntakeJam)
    {
      if (fabs(Intake1.velocity(rpm)) < 5 && Intake1.current() > 2.5)
      {
        // Reversa breve para liberar
        Intake1.spin(reverse, 100, percent);
        wait(150, msec);

        // Breif reverse to realease
        Intake1.spin(forward, 100, percent);
        // Forward
        wait(20, msec);
      }
      else
      {
        Intake1.spin(forward, 100, percent);
      }
    }

    wait(20,msec);
  }

}

/**
 * The expected behavior is to return to the start position.
 */
//Command Key
//chassis.drive_to_point(-0.1,-1.8);
// chassis.drive_max_voltage = 5;
// chassis.set_coordinates(0, 0, 0);
// Intake1.spin(forward, 100, pct);
// MatchLoad.set(true);
// chassis.set_drive_exit_conditions(1.5, 100, 900);
// chassis.set_turn_exit_conditions(1, 100, 3000);
// chassis.heading_max_voltage = 7;
// chassis.drive_to_pose(-2,-68,130);
// wait(650,msec);
// chassis.drive_with_voltage(-1, -1);
// chassis.drive_min_voltage = 0;

void drive_test(){

  // Starting
  Descore.set(false);
  odom_constants();
  thread matchloadThread(matchloadTask);

  // First set of balls
  chassis.drive_max_voltage = 9;
  flagMatchLoad2 = true;
  chassis.set_coordinates(0, 0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(1, 100, 900);
  chassis.drive_distance(14);
  chassis.set_turn_exit_conditions(1, 100, 300);
  chassis.turn_to_angle(27);
  chassis.set_drive_exit_conditions(1, 100, 900);
  chassis.drive_distance(16);
  flagIntakeJam = true;

  // Matchloader
  chassis.turn_to_angle(127);
  chassis.drive_with_voltage(5, 5);
  wait(900,msec);
  chassis.set_turn_exit_conditions(1, 100, 300);
  chassis.turn_to_angle(180);
  chassis.drive_max_voltage = 4;
  chassis.set_drive_exit_conditions(1, 100, 900);
  chassis.set_turn_exit_conditions(1, 100, 300);
  chassis.drive_to_pose(30,-20, 180);
  wait(300,msec);
  chassis.drive_max_voltage = 10;
  
  // Scoring
  chassis.heading_max_voltage = 7;
  chassis.set_drive_exit_conditions(1, 100, 900);  
  chassis.drive_to_point(30,40);
  chassis.set_turn_exit_conditions(1, 100, 300);
  chassis.drive_to_pose(30,-20, 180);
  flagIntakeJam = true;
  Intake1.spin(forward, 100, pct);
  Intake2.spin(forward, 100, pct);
  wait(2000,msec);

  // Push
  chassis.drive_with_voltage(5, 5);
  wait(400,msec);
  chassis.set_turn_exit_conditions(1, 100, 300);
  chassis.turn_to_angle(90);
  chassis.drive_with_voltage(5, 5);
  wait(400,msec);
  chassis.set_turn_exit_conditions(1, 100, 300);
  chassis.turn_to_angle(184);
  chassis.drive_with_voltage(-5, -5);
  wait(400,msec);
  
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  odom_constants();
  chassis.drive_max_voltage = 12;
  chassis.set_coordinates(0, 0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 100, pct);
  chassis.drive_distance(13.5);
  //chassis.drive_max_voltage = 12;

  chassis.turn_to_angle(-27);
  chassis.drive_distance(6.5);
  MatchLoad.set(true);

  chassis.drive_distance(10);
  chassis.turn_to_angle(-140);
  chassis.drive_distance(-15.25);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 50, pct);
  CenterGoal.set(true);
  wait(4000,msec);
  
  chassis.drive_distance(25);
  


  

  /*
 
  // Drive to Matchloader
  chassis.drive_max_voltage = 8;
   Intake1.spin(forward, 40, pct);
  Intake2.spin(reverse, 40, pct);
  chassis.drive_distance(29);//30
  */
  
  /*
  //matchloader
  chassis.turn_to_angle(180);
  chassis.drive_distance(15);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 100, pct);
  MatchLoad.set(true);
  chassis.drive_max_voltage = 6;
  wait(100,msce);

  */

  /*
  chasis.drive_distance(-25);
  Intake1.spin(forward, 100, pct);
  Intake2/'.spin(forward, 100, pct);
  */
//DONE TILL
  /*
  Intake1.spin(forward, -100, pct);
  Intake2.spin(reverse, -100, pct);
  wait(100,msec);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 50, pct);
  chassis.drive_distance(20.1, 180);
  wait(700,msec);
    chassis.drive_max_voltage = 10;
  /
  chassis.drive_distance(-5, 180);
  chassis.turn_to_angle(-185);
  chassis.drive_distance(25);
  MatchLoad.set(false);

  Intake1.spin(forward, 100, pct);
  Intake2.spin(forward, 100, pct);
  wait(2300,msec);
  Descore.set(false);
  Intake1.spin(forward, 0, pct);
  Intake2.spin(forward, 0, pct);
  chassis.drive_distance(8);
  chassis.turn_to_angle(90);
  chassis.drive_distance(10.5);
  chassis.turn_to_angle(-5);
  chassis.drive_max_voltage = 5;

  chassis.drive_distance(21);
  wait(3000,msec);
*/





}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  Descore.set(false);

  //first set of balls
  odom_constants();
  chassis.drive_max_voltage = 6;
  chassis.set_coordinates(0, 0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 20, pct);
  chassis.drive_distance(14);
  chassis.drive_max_voltage = 12;

  chassis.turn_to_angle(-27);
  MatchLoad.set(true);

  chassis.drive_distance(16);
  MatchLoad.set(true);


  //matchloader
  chassis.drive_max_voltage = 6;
   Intake1.spin(forward, 40, pct);
  Intake2.spin(reverse, 40, pct);
  chassis.turn_to_angle(-127);
  chassis.drive_distance(30.5);//30

  chassis.turn_to_angle(180);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 100, pct);
  MatchLoad.set(true);
  chassis.drive_max_voltage = 4;
  
  Intake1.spin(forward, -100, pct);
  Intake2.spin(reverse, -100, pct);
  wait(100,msec);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 50, pct);
  chassis.drive_distance(20, 180);
  wait(300,msec);
    chassis.drive_max_voltage = 10;
  //scoring
  chassis.turn_to_angle(-189);
  //   Intake1.spin(forward, 0, pct);
  // Intake2.spin(reverse, 0, pct);
  //chassis.set_coordinates(0, 0, 0);

  //chassis.drive_to_point(-0.1,-1.8);


chassis.drive_distance(-28);
  // Intake1.spin(forward, 0, pct);
  // Intake2.spin(reverse, 0, pct);
  // chassis.turn_to_angle(189);
  // chassis.drive_distance(-28);
  MatchLoad.set(false);
  Intake1.spin(reverse, 50, pct);
  Intake2.spin(reverse, 50, pct);
  wait(200,msec);
  Intake1.spin(reverse,60, pct);
  Intake2.spin(reverse, 80, pct);
  wait(100,msec);

  Intake1.spin(forward, 100, pct);
  Intake2.spin(forward, 100, pct);
  chassis.drive_distance(-3);
  wait(5000,msec);
  Descore.set(false);
  //Intake1.spin(forward, 0, pct);
  //Intake2.spin(forward, 0, pct);
  // chassis.drive_distance(8);
  // chassis.turn_to_angle(-90);
  // chassis.drive_distance(10.5);
  // chassis.turn_to_angle(0);
  // chassis.drive_max_voltage = 5;

  // chassis.drive_distance(21);
  // wait(3000,msec);


  //chassis.drive_max_voltage = 10;
  
  //chassis.set_coordinates(0, 0, 0);
  //chassis.turn_to_point(10,0, -90);
 
  //Descore.set(true);

  //chassis.drive_distance(13);




//    MatchLoad.set(true);
//   Intake2.spin(forward, 100, pct);


}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  Intake1.spin(forward, 100, pct);

}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}
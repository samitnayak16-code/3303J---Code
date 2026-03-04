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
bool Autocontrol = false;
bool Scoring1 = true;

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
      wait(1000,msec);           // wait 800 milliseconds
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
    if(Scoring1)
    {
      if (fabs(Intake2.velocity(rpm)) < 5 )
      {
        chassis.drive_distance(0.2);
        Intake1.spin(forward, 100, percent);
        Intake2.spin(forward, 100, percent);
        

      }
      else
      {
        Intake1.spin(forward, 100, percent);
        Intake2.spin(forward, 100, percent);

      }
    }
    if(flagIntakeJam)
    {
      if (fabs(Intake1.velocity(rpm)) < 5)
      {
        // Reversa breve para liberar
        Intake1.spin(reverse, 100, percent);
        wait(150, msec);

        //Brief reverse to release
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


void RightSide4_Push(){
  Descore.set(false);
  odom_constants();
  thread matchloadThread(matchloadTask);
  // First set of balls
  chassis.drive_max_voltage = 6;
  chassis.set_coordinates(0, 0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(0.5, 100, 5000);
  chassis.drive_distance(13);
  chassis.set_turn_exit_conditions(0.5, 100, 400);
  chassis.turn_to_angle(27);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(0.5, 100, 900);
  chassis.drive_distance(15);
  flagIntakeJam = true;
  chassis.set_turn_exit_conditions(0.5, 100, 600);
  chassis.turn_to_angle(125);
  
  chassis.set_turn_exit_conditions(1, 100, 300);
  MatchLoad.set(false);
  chassis.drive_distance(26.5);
  chassis.set_turn_exit_conditions(1, 300, 600);
  chassis.turn_to_angle(180);

  // Scoring
  chassis.set_drive_exit_conditions(1, 100, 900);  
  chassis.drive_distance(-15);
  chassis.drive_distance(1);

  chassis.drive_with_voltage(0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(forward, 100, pct);
  flagIntakeJam = true;
  Scoring1 = true;
  wait(1800,msec);

  // Push
  chassis.set_drive_exit_conditions(1.5, 300, 700);
  chassis.set_turn_exit_conditions(1, 300, 700);
  chassis.drive_distance(7);
  chassis.turn_to_angle(90);
  chassis.drive_distance(13.7);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-20);
  chassis.drive_with_voltage(0, 0);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void LeftSide4_Push(){
  Descore.set(false);
  odom_constants();
  thread matchloadThread(matchloadTask);
  // First set of balls
  chassis.drive_max_voltage = 10;
  chassis.set_coordinates(0, 0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(0.5, 100, 900);
  chassis.drive_distance(14.2);
  chassis.set_turn_exit_conditions(0.5, 100, 400);
  chassis.turn_to_angle(-25.9);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(0.5, 100, 900);
  chassis.drive_distance(16);
  flagIntakeJam = true;

  // Matchloader
  chassis.set_turn_exit_conditions(0.5, 100, 600);
  chassis.turn_to_angle(-140);
  chassis.set_drive_exit_conditions(0.5, 100, 2020);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(40);
  chassis.set_turn_exit_conditions(1, 300, 800);
  chassis.turn_to_angle(180);


 // --- Match Loader ---
  chassis.set_drive_exit_conditions(1, 100, 1200);
  chassis.drive_max_voltage = 10;
  chassis.drive_distance(7, 180);
  chassis.drive_with_voltage(4, 4);
  wait(800, msec);

  // --- Scoring ---
  chassis.set_drive_exit_conditions(1, 100, 1400);
  chassis.drive_distance(-30,176);
  chassis.drive_distance(0.9,180);
  MatchLoad.set(false);
  chassis.drive_with_voltage(0, 0);
  Intake1.spin(reverse, 100, percent);
  wait(150, msec);
  Intake1.spin(forward, 100, percent);
  Intake2.spin(forward, 100, pct);
  Scoring1 = true;
  wait(2500, msec);

  // --- Push / Endgame ---
  chassis.set_turn_exit_conditions(1, 300, 900);
  chassis.set_turn_exit_conditions(1, 300, 800);
  chassis.drive_with_voltage(0, 8);
  wait(350, msec);
  chassis.drive_distance(10);
  chassis.set_turn_exit_conditions(1, 300, 700);
  chassis.turn_to_angle(180);
  chassis.set_turn_exit_conditions(1, 300, 2000);
  chassis.drive_with_voltage(-5, -5);
  wait(1200, msec);
  chassis.drive_with_voltage(0, 0);


}

/**
 * Should swing in a fun S shape.
 */

void RightSide7_Push(){
Descore.set(false);
odom_constants();
thread matchloadThread(matchloadTask);

// --- First Ball Intake ---
chassis.drive_max_voltage = 10;
chassis.set_coordinates(0, 0, 0);
Intake1.spin(forward, 100, pct);
Intake2.spin(reverse, 20, pct);
chassis.set_drive_exit_conditions(0.5, 100, 600);
chassis.drive_distance(13);
chassis.set_turn_exit_conditions(0.5, 100, 400);
chassis.turn_to_angle(25.9);

// --- Match Load Pickup ---
MatchLoad.set(true);
chassis.set_drive_exit_conditions(0.5, 100, 700);
chassis.drive_distance(15);
flagIntakeJam = true;

// --- Navigate to Scoring Zone ---
chassis.set_turn_exit_conditions(0.5, 100, 800);
chassis.turn_to_angle(140);
chassis.set_drive_exit_conditions(0.5, 100, 1220);
chassis.drive_max_voltage = 8;
chassis.drive_distance(37.3);
chassis.set_turn_exit_conditions(1, 300, 800);
chassis.turn_to_angle(180);


// // --- Match Loader ---
chassis.set_drive_exit_conditions(1, 100, 1200);
chassis.drive_max_voltage = 10;
chassis.drive_distance(7, 183);
chassis.drive_with_voltage(4, 4);
wait(800, msec);

// --- Scoring ---
chassis.set_drive_exit_conditions(1, 100, 1400);
chassis.drive_distance(-30,180);
chassis.drive_distance(0.8,180);
MatchLoad.set(false);
chassis.drive_with_voltage(0, 0);
flagIntakeJam = true;
Intake2.spin(forward, 100, pct);
Scoring1 = true;
wait(2500, msec);

// --- Push / Endgame ---
chassis.set_turn_exit_conditions(1, 300, 900);
chassis.set_turn_exit_conditions(1, 300, 1000);
chassis.drive_with_voltage(0, 8);
wait(350, msec);
chassis.drive_distance(12);
chassis.set_turn_exit_conditions(1, 300, 700);
chassis.turn_to_angle(180);
chassis.set_turn_exit_conditions(1, 300, 2000);
chassis.drive_with_voltage(-5, -5);
wait(1200, msec);
chassis.drive_with_voltage(0, 0);


}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */


void LeftSide7_Push(){
 Descore.set(false);
  odom_constants();
  thread matchloadThread(matchloadTask);
  // First set of balls
  chassis.drive_max_voltage = 10;
  chassis.set_coordinates(0, 0, 0);
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 20, pct);
  chassis.set_drive_exit_conditions(0.5, 100, 900);
  chassis.drive_distance(14);
  chassis.set_turn_exit_conditions(0.5, 100, 400);
  chassis.turn_to_angle(-25.9);
  MatchLoad.set(true);
  chassis.set_drive_exit_conditions(0.5, 100, 900);
  chassis.drive_distance(16);
  flagIntakeJam = true;

  // Matchloader
  chassis.set_turn_exit_conditions(0.5, 100, 600);
  chassis.turn_to_angle(-127);
  
  chassis.set_drive_exit_conditions(1, 100, 1000);
  MatchLoad.set(false);
  chassis.drive_distance(27);
  chassis.set_turn_exit_conditions(1, 300, 400);
  chassis.turn_to_angle(180);

  // Scoring
  chassis.set_drive_exit_conditions(1, 100, 900);  
  chassis.drive_distance(-15);
chassis.drive_distance(0.8,180);
MatchLoad.set(false);
chassis.drive_with_voltage(0, 0);
flagIntakeJam = true;
Intake2.spin(forward, 100, pct);
Scoring1 = true;
wait(1800, msec);

// --- Push / Endgame ---
chassis.set_turn_exit_conditions(1, 300, 900);
chassis.set_turn_exit_conditions(1, 300, 1000);
chassis.drive_with_voltage(0, 8);
wait(350, msec);
chassis.drive_distance(11.5);
chassis.set_turn_exit_conditions(1, 300, 700);
chassis.turn_to_angle(180);
chassis.set_turn_exit_conditions(1, 300, 2000);
chassis.drive_with_voltage(-5, -5);
wait(1200, msec);
chassis.drive_with_voltage(0, 0);


}
void LeftSide4_3(){
  Descore.set(false);
odom_constants();
thread matchloadThread(matchloadTask);

// --- First Ball Intake ---
chassis.drive_max_voltage = 10;
chassis.set_coordinates(0, 0, 0);
Intake1.spin(forward, 100, pct);
Intake2.spin(reverse, 20, pct);
chassis.set_drive_exit_conditions(0.5, 100, 600);
chassis.drive_distance(13.5);
chassis.set_turn_exit_conditions(0.5, 100, 400);
chassis.turn_to_angle(-25.9);

// --- Match Load Pickup ---
MatchLoad.set(true);
chassis.set_drive_exit_conditions(0.5, 100, 700);
chassis.drive_distance(15);
flagIntakeJam = true;

// --- Navigate to Scoring Zone ---
chassis.set_turn_exit_conditions(0.5, 100, 900);
chassis.turn_to_angle(-136);
chassis.set_drive_exit_conditions(0.5, 100, 1920);
chassis.drive_max_voltage = 5;
chassis.drive_distance(-17.8);
chassis.set_turn_exit_conditions(1, 300, 800);
Intake2.spin(reverse, 100, pct);
chassis.drive_with_voltage(0, 0);
wait(800, msec);
Intake2.spin(reverse, 20, pct);
chassis.set_drive_exit_conditions(0.5, 100, 3550);
chassis.drive_max_voltage = 8;
chassis.drive_distance(54);
Intake2.spin(reverse, 20, pct);


// // // --- Match Loader ---
chassis.set_turn_exit_conditions(1, 300, 900);
chassis.turn_to_angle(180);
chassis.set_drive_exit_conditions(1, 100, 1200);
chassis.drive_max_voltage = 5;
chassis.drive_distance(16, 180);
chassis.drive_with_voltage(4, 4);
wait(550, msec);

// --- Scoring ---
chassis.set_drive_exit_conditions(1, 100, 1400);
chassis.drive_max_voltage = 10;
chassis.drive_distance(-30,182);
chassis.drive_distance(0.8,180);
MatchLoad.set(false);
chassis.drive_with_voltage(0, 0);
flagIntakeJam = true;
Intake2.spin(forward, 100, pct);
Scoring1 = true;
wait(2500, msec);

// --- Push / Endgame ---
chassis.set_turn_exit_conditions(1, 300, 900);
chassis.set_turn_exit_conditions(1, 300, 1000);
chassis.drive_with_voltage(0, 8);
wait(350, msec);
chassis.drive_distance(12.5);
chassis.set_turn_exit_conditions(1, 300, 700);
chassis.turn_to_angle(180);
chassis.set_turn_exit_conditions(1, 300, 2000);
chassis.drive_with_voltage(-5, -5);
wait(1200, msec);
chassis.drive_with_voltage(0, 0);

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

#include "vex.h"
#define RED 1
#define BLUE 2
using namespace vex;

competition Competition;

int ALLIANCE_COLOR = RED;

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
TANK_TWO_ENCODER,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(Left1,Left2,Left3),

//Right Motors:
motor_group(Right1,Right2,Right3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT10,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,//360/1.015

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT5,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT21,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0.02,//0.02

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT2,

//Sideways tracker diameter (reverse to make the direction switch):
2,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
3.02//3.02

);

int current_auton_selection = 1;
bool auto_started = false;
double lastPos = 0;
bool movedUp = false;
bool movedDown = false;
bool blockerstate = false;
void pre_auton() 
{
  vexcodeInit();
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  ColorSortSensor.setLight(ledState::on);
  ColorSortSensor.setLightPower(100);
  Brain.Screen.setFont(vex::fontType::mono40);
  Intake2.setPosition(0,degrees);
  int last_selection = -1;
  while(!auto_started &&(Competition.isCompetitionSwitch() || Competition.isFieldControl()) )
  {
    if (Competition.isCompetitionSwitch() || Competition.isFieldControl()) 
    {
      if(ALLIANCE_COLOR == RED)
      {
        Brain.Screen.clearScreen(red);
        Brain.Screen.setFillColor(vex::color::red);
      }
      else
      {
        Brain.Screen.clearScreen(blue);
        Brain.Screen.setFillColor(vex::color::blue);
      }
      Brain.Screen.printAt(5, 40, "Battery: %d   %", Brain.Battery.capacity());
      
      if(current_auton_selection != last_selection)
      {
        last_selection = current_auton_selection; 
        Controller1.Screen.clearLine(3);
      }
      Controller1.Screen.setCursor(3, 1);
      switch(0)
      {
        case 0:
          Brain.Screen.printAt(5, 80, "RightSide 7 + Push");
          Controller1.Screen.print("RightSide 7 + Push");
          break;
        case 1:
          Brain.Screen.printAt(5, 80, "LeftSide 7 + Push");
          Controller1.Screen.print("LeftSide 7 + Push");
          break;
        case 2:
          Brain.Screen.printAt(5, 80, "RIGHT Low Goal+Push");
          Controller1.Screen.print("RIGHT Low Goal+Push");
          break;
        case 3:
          Brain.Screen.printAt(5, 80, "RIGHT Low Goal");
          Controller1.Screen.print("RIGHT Low Goal");
          break;
        case 4:
          Brain.Screen.printAt(5, 80, "RIGHT RUSH+Push");
          Controller1.Screen.print("RIGHT RUSH+Push");
          break;
        case 5:
          Brain.Screen.printAt(5, 80, "RIGHT RUSH");
          Controller1.Screen.print("RIGHT RUSH");
          break;
        case 6:
          Brain.Screen.printAt(5, 80, "RIGHT SUPER RUSH");
          Controller1.Screen.print("RIGHT SUPER RUSH");
          break;
        case 7:
          Brain.Screen.printAt(5, 80, "LEFT Middle+Push");
          Controller1.Screen.print("LEFT Middle+Push");
          break;
        case 8:
          Brain.Screen.printAt(5, 80, "LEFT Middle");
          Controller1.Screen.print("LEFT Middle");
          break;
        case 9:
          Brain.Screen.printAt(5, 80, "LEFT RUSH+Push");
          Controller1.Screen.print("LEFT RUSH+Push");
          break;
        case 10:
          Brain.Screen.printAt(5, 80, "LEFT RUSH");
          Controller1.Screen.print("LEFT RUSH");
          break;
        case 11:
          Brain.Screen.printAt(5, 80, "LEFT SUPER RUSH");
          Controller1.Screen.print("LEFT SUPER RUSH");
          break;
        case 12:
          Brain.Screen.printAt(5, 80, "Skills");
          Controller1.Screen.print("Skills");
          break;
      }
      // --- Manual movement control of the Intake2 motor ---
      double currentPos = Intake2.position(degrees);
      double delta = currentPos - lastPos;

      // Detection of rising or falling edge with threshold to avoid noise
      const double threshold = 6; // Adjust if you want it to be more or less sensitive.

      if (delta > threshold && !movedUp) 
      {
      // Movement in one direction (for example, forward)
        current_auton_selection++;
        movedUp = true;
        movedDown = false;
      }
      else if (delta < -threshold && !movedDown) 
      {
        // Movement in the other direction
        current_auton_selection--;
        movedDown = true;
        movedUp = false;
      }
        // When the engine stops, we reset the flags.
      if (fabs(delta) < 10) //
      {
        movedUp = false;
        movedDown = false;
      }
      lastPos = currentPos;

      if (Brain.Screen.pressing()) 
      {
        while(Brain.Screen.pressing())  
          wait(10,msec); 
        if (Brain.Screen.xPosition()> 480/2)  
            current_auton_selection++;
        else        
            current_auton_selection--;
      }
      if (current_auton_selection > 12) current_auton_selection = 0;
      if (current_auton_selection < 0) current_auton_selection = 12;
      wait(10,msec);
    }
  }
}
void autonomous(void) 
{
  chassis.drive_stop(hold);
  auto_started = true;
  switch(0)
  { 
    case 0:
    //  tank_odom_test();
      drive_test();
      break;
    case 1:         
      turn_test();
       //break;
    // case 2:
    //   RightSide_1LowGoalPush();  
    //   break;
    // case 3:
    //   RightSide_1LowGoal();  
    //   break;
    // case 4:
    //   RightSideRushPush();
    //   break;
    // case 5:
    //   RightSideRush();
    //   break;
    // case 6:
    //   RightSideSUPERRush();
    //   break;
    // case 7:
    //   LeftSide_MiddleGoalPush();
    //   break;
    // case 8:
    //   LeftSide_MiddleGoal();
    //   break;
    // case 9:
    //   LeftSideRushPush();
    //   break;
    // case 10:
    //   LeftSideRush();
    //   break;
    // case 11:
    //   LeftSideSUPERRush();
    //   break;
    // case 12:
    //   Skills();
    //   break;
 }
}

bool colorSort = true;
bool descoreState = false;   // Initial state
bool downPressed = false;    // To prevent bouncing

void usercontrol(void) 
{
  chassis.drive_stop(coast);
  Brain.Screen.setFillColor(vex::color::yellow);
  // flagIntakeJam = false;
  // Autocontrol = false;
  while (true) 
  {
      // Controller1.Screen.clearLine(1);
      // Controller1.Screen.setCursor(1, 1);
      // Controller1.Screen.print("colorSort: %s", colorSort ? "ON" : "OFF");

      // if (Controller1.ButtonB.pressing()) 
      // {
      // // Espera a que se suelte el botón para evitar múltiples activaciones rápidas
      // while (Controller1.ButtonB.pressing()) {
      //   wait(10, msec);
      // }
      // // Cambia el estado (toggle)
      // colorSort = !colorSort;

      // // Imprime el estado en la pantalla del control
  
      // }


    //Brain.Screen.clearScreen(yellow);
    double x = chassis.get_X_position();
    double y = chassis.get_Y_position();
    double heading = chassis.get_absolute_heading();
// Print to the VEXcode console
    printf("X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);

    if(!IMU.isCalibrating())
    {
      Brain.Screen.setFillColor(vex::color::yellow);
      Brain.Screen.clearScreen(yellow);
      Brain.Screen.setPenColor(vex::color::black);
      Brain.Screen.setFont(vex::fontType::mono60);
      if(ALLIANCE_COLOR == RED)
      {
        Brain.Screen.setPenColor(vex::color::red);
        Brain.Screen.printAt(5,40, "RED ALLIANCE");
      }
      else
      {
        Brain.Screen.setPenColor(vex::color::blue);
        Brain.Screen.printAt(5,40, "BLUE ALLIANCE");       
      }
      Brain.Screen.setPenColor(vex::color::black);
      Brain.Screen.setFont(vex::fontType::mono40);
      Brain.Screen.printAt(5,80, "X:%.2f", chassis.get_X_position());
      Brain.Screen.printAt(320,80, "Y:%.2f", chassis.get_Y_position());
      Brain.Screen.printAt(5,120, "H:%.2f", chassis.get_absolute_heading());
    }
    // Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    // Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    chassis.control_arcade();

    if (Controller1.ButtonX.pressing()) {
      if (!downPressed) {
        descoreState = !descoreState;      // Change the state
        Descore.set(descoreState);         // Update output to the solenoid
        downPressed = true;                // Mark that this pressure has already been counted
      }
    } else {
      downPressed = false;                 // The button is released -> allows the next toggle
    }


    if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) 
    {
        Intake1.spin(fwd, 0, pct);
        Intake2.spin(fwd, 0, pct);
        if (Controller1.ButtonR1.pressing()) 
            Descore.set(true);
        else
            Descore.set(false);
    } 
    else if (Controller1.ButtonL1.pressing() && Controller1.ButtonR1.pressing()) 
    { 
      MatchLoad.set(true);
      Intake1.spin(forward, 100, pct);
      // if(colorSort == true)
      // {
        // if(ColorSortSensor.hue() > 200 && ColorSortSensor.hue() < 250 && ALLIANCE_COLOR == RED && ColorSortSensor.isNearObject() )    
        //   Intake2.spin(forward, 100, pct);
        // else if(ColorSortSensor.hue() > 0 && ColorSortSensor.hue() < 20 && ALLIANCE_COLOR == BLUE && ColorSortSensor.isNearObject())
        //   Intake2.spin(forward, 100, pct);
        // else
        //   Intake2.spin(reverse, 20, pct);
      // }
      // else
       Intake2.spin(reverse, 20, pct);
 
    } 
    else if (Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing()) 
    {
      Intake1.spin(fwd, 100, pct);
      Intake2.spin(reverse, 20, pct);
  
    } 
    else if (Controller1.ButtonL2.pressing() && !Controller1.ButtonL1.pressing()) 
    {
      Intake1.spin(reverse, 100,pct);
      Intake2.spin(reverse, 100,pct);
    } 
    else if (Controller1.ButtonR1.pressing()) 
    {
      // if(colorSort == true)
      // {
        // if(ColorSortSensor.hue() > 200 && ColorSortSensor.hue() < 250 && ALLIANCE_COLOR == RED && ColorSortSensor.isNearObject() )// azul
        // {
        //   CenterGoal.set(true);
        //   //wait(50,msec);
        //   Intake2.spin(reverse, 100,pct);
        // }
        // else if(ColorSortSensor.hue() > 0 && ColorSortSensor.hue() < 20 && ALLIANCE_COLOR == BLUE && ColorSortSensor.isNearObject() )
        // {
        //   CenterGoal.set(true);
        //   //wait(50,msec);
        //   Intake2.spin(reverse, 100,pct);
        // }
        // else
        // {
        //   CenterGoal.set(false);
        //   Intake2.spin(forward, 100,pct);    
        // }
      // }
      // else
      // {
        Intake2.spin(forward, 100,pct); 
      // }
    
      Intake1.spin(fwd, 100,pct);
    } 
    else if (Controller1.ButtonR2.pressing()) 
    {
      Intake1.spin(forward, 100, pct);
      Intake2.spin(reverse, 50, pct);
      CenterGoal.set(true);
      
    } 
    else if (Controller1.ButtonUp.pressing()) 
    {

      Blocker.set(blockerstate);
      blockerstate = !blockerstate;
    }
    else 
    {
      Intake1.spin(forward, 0, pct);
      Intake2.spin(forward, 0,pct);
      CenterGoal.set(false);
      MatchLoad.set(false);
      Blocker.set(false);
      //Descore.set(true);
    }
    if (Controller1.ButtonLeft.pressing()) 
    {
        chassis.drive_with_voltage(12,12);
        wait(150,msec);
        chassis.drive_with_voltage(0,0);
        wait(300,msec);
        for (int i = 0; i <= 15; i++) 
        {
            Left1.spin(reverse, i * 9, pct);
            Left2.spin(reverse, i * 9, pct);
            Left3.spin(reverse, i * 9, pct);
            Right1.spin(reverse, i * 9, pct);
            Right2.spin(reverse, i * 9, pct);
            Right3.spin(reverse, i * 9, pct);
            wait(20,msec);
        }
        chassis.drive_with_voltage(0,0);
    }
    wait(20, msec); 

  }
}

int main() 
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) 
    wait(20, msec);
}

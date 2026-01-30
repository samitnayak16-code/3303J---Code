using namespace vex;

extern brain Brain;
extern controller Controller1;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;
extern motor Left1;
extern motor Left2;
extern motor Left3;
extern motor Right1;
extern motor Right2;
extern motor Right3;
extern motor Intake1;
extern motor Intake2;
extern inertial IMU;

//Add your devices below, and don't forget to do the same in robot-config.cpp:
extern digital_out CenterGoal;
extern digital_out MatchLoad;
extern digital_out Descore;
extern digital_out Blocker;
extern optical ColorSortSensor;


void  vexcodeInit( void );
#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();

void RightSide4_Push();
void LeftSide4_Push();
void RightSide7_Push();
void LeftSide7_Push();
void LeftSide4_3();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

#include "lemlib/api.hpp"
// #include "robot.h"
using namespace pros;
using namespace std;

#ifndef GLOBALS_H
#define GLOBALS_H

extern lemlib::Chassis chassis;
extern adi::DigitalOut clip1;
extern adi::DigitalOut clip2;
extern adi::DigitalOut stick;
extern Robot robot;
extern Controller master;

#endif
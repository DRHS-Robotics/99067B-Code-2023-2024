#include "lemlib/api.hpp"
#ifndef MOTORS_h
#define MOTORS_h
extern pros::Motor fl_drive; 
extern pros::Motor ml_drive;
extern pros::Motor tl_drive;
extern pros::Motor bl_drive;
extern pros::Motor fr_drive;
extern pros::Motor mr_drive;
extern pros::Motor tr_drive;
extern pros::Motor br_drive;
extern pros::Motor intake;
// extern pros::Motor climb;

extern pros::ADIDigitalOut PTO;
extern pros::ADIDigitalOut wings1;
extern pros::ADIDigitalOut wings2;
extern pros::ADIDigitalOut frontWings1;
extern pros::ADIDigitalOut frontWings2;
extern pros::ADIDigitalOut climbRelease;
extern pros::IMU imu;
// extern pros::IMU imu2;

extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;
extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateralController;
extern lemlib::ControllerSettings angularController;
extern lemlib::Chassis chassis; 
#endif
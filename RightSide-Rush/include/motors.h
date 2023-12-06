#ifndef MOTORS_h
#define MOTORS_h

extern pros::Motor ptoL_drive; 
extern pros::Motor tl_drive;
extern pros::Motor bl_drive;
extern pros::Motor ptoR_drive;
extern pros::Motor tr_drive;
extern pros::Motor br_drive;
extern pros::Motor intake1;
extern pros::Motor intake2;
extern pros::Motor tapa;
extern pros::Motor flywheel;


extern pros::ADIDigitalIn tapaSwitch;
extern pros::ADIDigitalOut PTO;
extern pros::ADIDigitalOut wings1;
extern pros::ADIDigitalOut wings2;
extern pros::ADIDigitalOut climbRelease;
extern pros::ADIDigitalOut matchLoad;

extern pros::IMU imu1;
extern pros::IMU imu2;

#endif
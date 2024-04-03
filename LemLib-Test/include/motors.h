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
extern pros::ADIDigitalOut matchLoad;
extern pros::ADIDigitalIn climbSwitch;

extern pros::IMU imu1;
extern pros::IMU imu2;
extern pros::Rotation vertEncd;
extern pros::Rotation horiEncd;

extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;
extern lemlib::Drivetrain_t drivetrain;
extern lemlib::TrackingWheel vertTracking;
extern lemlib::TrackingWheel horiTracking;
extern lemlib::OdomSensors_t sensors;
extern lemlib::ChassisController_t lateralController;
extern lemlib::ChassisController_t angularController;
extern lemlib::Chassis chassis; 


#endif
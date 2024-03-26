#include "main.h"
pros::Motor fl_drive(20, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor ml_drive(7, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);//top left
pros::Motor bl_drive(9, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tl_drive(20, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS); //5.5 motor
pros::Motor fr_drive(3, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor mr_drive(1, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);//top right
pros::Motor br_drive(2, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tr_drive(20, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS); //5.5 motor
pros::Motor slapper(13, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor climb1(12, pros::E_MOTOR_GEARSET_36, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor climb2(15, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake(14, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::MotorGroup left_side_motors({fl_drive, ml_drive, bl_drive, tl_drive});
pros::MotorGroup right_side_motors({fl_drive, mr_drive, bl_drive, tr_drive});
// pros::ADIDigitalIn tapaSwitch('H');
pros::ADIDigitalOut wings1('C');
pros::ADIDigitalOut wings2('F');
pros::ADIDigitalOut frontWings1('A');
pros::ADIDigitalOut frontWings2('B');
pros::ADIDigitalOut climbRelease('E');
// pros::ADIDigitalOut matchLoad('D');
// pros::ADIDigitalOut PTO('E');
// pros::ADIDigitalIn climbSwitch('B');
pros::IMU imu1(10);
pros::IMU imu2(11);
pros::Rotation climbRot(12);
pros::Optical optical_slapper(6);
// pros::ADIDigitalOut climbRelease('D');

int sgn(double num);
double returnThetaInRange(double thetaAngle);
void drive(double left, double right);
void initializeTapaTask();
void control_flywheel_fn();
void lift_macro();
void PTO_Drive(double left, double right);
int sgn(double num);
void resetPos();
double position();
void setWings();
void control_turn(double target, double maxPower, double turnkI, double turnkD, bool accuracy, bool needed);
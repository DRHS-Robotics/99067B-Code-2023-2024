#include "main.h"
#include "lemlib/api.hpp"
pros::Motor fl_drive(2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor ml_drive(14, pros::E_MOTOR_GEARSET_06, false);
pros::Motor tl_drive(11, pros::E_MOTOR_GEARSET_18, true);//top left
pros::Motor bl_drive(13, pros::E_MOTOR_GEARSET_06, true);
pros::Motor fr_drive(9, pros::E_MOTOR_GEARSET_06, true);
pros::Motor mr_drive(17, pros::E_MOTOR_GEARSET_06, true);
pros::Motor tr_drive(19, pros::E_MOTOR_GEARSET_18, false);//top right
pros::Motor br_drive(20, pros::E_MOTOR_GEARSET_06, false);
pros::Motor intake(3, pros::E_MOTOR_GEARSET_18, 0);

// pros::ADIDigitalIn tapaSwitch('H');
pros::ADIDigitalOut wings1('C');
pros::ADIDigitalOut wings2('D');
pros::ADIDigitalOut frontWings1('A');
pros::ADIDigitalOut frontWings2('F');
pros::ADIDigitalOut climbRelease('B');
pros::ADIDigitalOut PTO('E');
//Lemlib
pros::IMU imu(16);

pros::MotorGroup left_side_motors({fl_drive, ml_drive, bl_drive, tl_drive});
pros::MotorGroup right_side_motors({fl_drive, mr_drive, bl_drive, tr_drive});

pros::Task* liftTask = nullptr; 


// lemlib::Drivetrain_t drivetrain{
//     &left_side_motors, // left drivetrain motors
//     &right_side_motors, // right drivetrain motors
//     10, // track width(how wide between the wheels)
//     lemlib::Omniwheel::OLD_4,
//     343, // wheel rpm
//     2 //Chase Power
// };

lemlib::Drivetrain drivetrain{
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    11, // track width(how wide between the wheels)
    lemlib::Omniwheel::NEW_325,
    450, // wheel rpm
    0 //Chase Power
};

lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu, // inertial sensor
};

lemlib::ControllerSettings linearController(
    20, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    300, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    40 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(
    7, // proportional gain (kP)
    0, // integral gain (kI)
    0, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    0, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
/////////////////////////////////////////////

int sgn(double num);
void drive(double left, double right);
void lift_macro();
int sgn(double num);
void resetPos();
double position();
void screen();
void lift_macro();
void resetPos();
double velocity();

bool PTO_State = false;
bool ratchetState = false;


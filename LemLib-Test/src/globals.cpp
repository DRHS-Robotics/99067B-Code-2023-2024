#include "main.h"
#include "lemlib/api.hpp"
pros::Motor fl_drive(2, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor ml_drive(14, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor tl_drive(11, pros::E_MOTOR_GEARSET_18, 1);//top left
pros::Motor bl_drive(13, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor fr_drive(9, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor mr_drive(17, pros::E_MOTOR_GEARSET_06, 1);//top right
pros::Motor tr_drive(19, pros::E_MOTOR_GEARSET_18, 0);//top right
pros::Motor br_drive(20, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor intake(15, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_COUNTS);

////////////////////////////////////
//Lemlib
pros::IMU imu(16);
// pros::IMU imu(20);

// pros::IMU imu1(21);

pros::MotorGroup left_side_motors({fl_drive, ml_drive, bl_drive, tl_drive});
pros::MotorGroup right_side_motors({fl_drive, mr_drive, bl_drive, tr_drive});

pros::Task liftTask = nullptr; 


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
    2 //Chase Power
};
//lemlib::TrackingWheel vertTracking(&vertEncd, 2.75, 4.3, 5/3); //2.75 inch diameter, 4.3 inches from the center left and right, 5:3 gear ratio
//lemlib::TrackingWheel horiTracking(&horiEncd, 2.75, -4.3, 5/3); //2.75 inch diameter, 4.3 inches from the center forwards and backwards, 5:3 gear ratio
lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu, // inertial sensor
};
lemlib::ControllerSettings linearController(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(
    2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
/////////////////////////////////////////////

// pros::ADIDigitalIn tapaSwitch('H');
pros::ADIDigitalOut wings1('C');
pros::ADIDigitalOut wings2('F');
pros::ADIDigitalOut frontWings1('A');
pros::ADIDigitalOut frontWings2('B');
pros::ADIDigitalOut climbRelease('E');

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

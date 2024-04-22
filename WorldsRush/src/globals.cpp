#include "main.h"
#include "lemlib/api.hpp"
pros::Motor fl_drive(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor ml_drive(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tl_drive(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);//top left
pros::Motor bl_drive(13, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor fr_drive(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor mr_drive(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tr_drive(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);//top right
pros::Motor br_drive(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
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
pros::MotorGroup right_side_motors({fr_drive, mr_drive, br_drive, tr_drive});

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
    50 //Chase Power
};

lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu, // inertial sensor
};

lemlib::ControllerSettings linearController {
    10, // kP
    0, // kI
    75, // kD
    3, // windup range
    1.5, // smallErrorRange
    100, // smallErrorTimeout
    4, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// turning PID
// lemlib::ControllerSettings angularController {
//     1.75, // kP
//     0, // kI
//     16, // kD
//     15, // windup range
//     1, // smallErrorRange
//     100, // smallErrorTimeout
//     3, // largeErrorRange
//     350, // largeErrorTimeout
//     0 // slew rate
// };
lemlib::ControllerSettings angularController {
    1.75, // kP
    0, // kI
    17, // kD
    15, // windup range
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    350, // largeErrorTimeout
    0 // slew rate
};

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
double returnThetaInRange();

bool PTO_State = false;
bool ratchetState = false;
int count = 0;
#include "main.h"
#include "lemlib/api.hpp"
// pros::Motor fl_drive(2, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor fl_drive(9, pros::E_MOTOR_GEARSET_06, 1);
// pros::Motor ml_drive(14, pros::E_MOTOR_GEARSET_06, 0);
// pros::Motor tl_drive(11, pros::E_MOTOR_GEARSET_18, 1);//top left
// pros::Motor bl_drive(13, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor bl_drive(16, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor fr_drive(10, pros::E_MOTOR_GEARSET_06, 0);
// pros::Motor fr_drive(9, pros::E_MOTOR_GEARSET_06, 1);
// pros::Motor mr_drive(17, pros::E_MOTOR_GEARSET_06, 1);//top right
// pros::Motor tr_drive(19, pros::E_MOTOR_GEARSET_18, 0);//top right
// pros::Motor br_drive(20, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor br_drive(17, pros::E_MOTOR_GEARSET_06, 0);
// pros::Motor tapa(17, pros::E_MOTOR_GEARSET_36, 0, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor slapper(13, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor climb1(12, pros::E_MOTOR_GEARSET_36, 1, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor climb2(15, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor flywheel(21, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);d
pros::Motor intake(15, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_COUNTS);

////////////////////////////////////
//Lemlib
// pros::IMU imu(16);
pros::IMU imu(20);

// pros::IMU imu1(21);

// pros::MotorGroup left_side_motors({fl_drive, ml_drive, bl_drive, tl_drive});
pros::MotorGroup left_side_motors({fl_drive, bl_drive});
// pros::MotorGroup right_side_motors({fl_drive, mr_drive, bl_drive, tr_drive});
pros::MotorGroup right_side_motors({fl_drive, bl_drive});

// lemlib::Drivetrain_t drivetrain{
//     &left_side_motors, // left drivetrain motors
//     &right_side_motors, // right drivetrain motors
//     11, // track width(how wide between the wheels)
//     3.25, // wheel diameter
//     450 // wheel rpm
// };
lemlib::Drivetrain_t drivetrain{
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10, // track width(how wide between the wheels)
    4, // wheel diameter
    343 // wheel rpm
};
//lemlib::TrackingWheel vertTracking(&vertEncd, 2.75, 4.3, 5/3); //2.75 inch diameter, 4.3 inches from the center left and right, 5:3 gear ratio
//lemlib::TrackingWheel horiTracking(&horiEncd, 2.75, -4.3, 5/3); //2.75 inch diameter, 4.3 inches from the center forwards and backwards, 5:3 gear ratio
lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu, // inertial sensor
};
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);
/////////////////////////////////////////////

// pros::ADIDigitalIn tapaSwitch('H');
pros::ADIDigitalOut wings1('C');
pros::ADIDigitalOut wings2('F');
pros::ADIDigitalOut frontWings1('A');
pros::ADIDigitalOut frontWings2('B');
pros::ADIDigitalOut climbRelease('E');

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
void screen();
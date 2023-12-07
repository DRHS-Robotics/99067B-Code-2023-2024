#include "main.h"
pros::Motor ptoL_drive(2, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tl_drive(1, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);//top left
pros::Motor bl_drive(3, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor ptoR_drive(7, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tr_drive(8, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);//top right
pros::Motor br_drive(9, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tapa(20, pros::E_MOTOR_GEARSET_36, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(6, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake2(5, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIDigitalIn tapaSwitch('H');
pros::ADIDigitalOut wings1('A');
pros::ADIDigitalOut wings2('F');
pros::ADIDigitalOut climbRelease('B');
pros::ADIDigitalOut matchLoad('D');
pros::IMU imu1(20);
pros::IMU imu2(11);
// pros::ADIDigitalOut climbRelease('D');

int sgn(double num);
double returnThetaInRange(double thetaAngle);
void drive(double left, double right);
void initializeTapaTask();
void control_flywheel_fn();
void PTO_Drive(double left, double right);
int sgn(double num);
void resetPos();
double position();
void setWings();
void control_turn(double target, double maxPower, double turnkI);

bool frontSlapaState = false;
bool backSlapaState = false;
bool flywheelOn = false;
int targetVoltage = 0;
bool reset = false;
const double maxtapaShoot = 1150.0;
tapaSpeed tapaSpeedControl;
pros::Task* tapaTask = nullptr;
pros::Task* wingsExpand = nullptr;
pros::Task* flywheelTask = nullptr;
bool wing1Expand = false;
bool wing2Expand = false;
bool bothWingsExpand = false;

DrivePID Drive;
ArcTurn arcTurn;

double arcIntegral = 0.0;
double arcDerivative = 0.0;
double arcLastError = 0.0;

double driveProportion = 0.0;
double driveDerivative = 0.0;
double driveLastError = 0.0;

double angleProportion = 0.0;
double angleDerivative = 0.0;
double angleLastError = 0.0;

double turnIntegral = 0.0;
double turnDerivative = 0.0;
double turnLastError = 0.0;

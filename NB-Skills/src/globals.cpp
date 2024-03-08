#include "main.h"
pros::Motor ptoL_drive(20, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tl_drive(7, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);//top left
pros::Motor bl_drive(9, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor ptoR_drive(3, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tr_drive(1, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);//top right
pros::Motor br_drive(2, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor tapa(17, pros::E_MOTOR_GEARSET_36, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor slapper(13, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor climb1(12, pros::E_MOTOR_GEARSET_36, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor climb2(15, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(21, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake2(14, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_COUNTS);
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
void control_turn(double target, double maxPower, double turnkP, bool needed);

bool frontSlapaState = false;
bool initialSlapaMovement = false;
bool backSlapaState = false;
bool flywheelOn = false;
int targetVoltage = 0;
bool reset = false;
const double maxtapaShoot = 1150.0;
tapaSpeed tapaSpeedControl;
pros::Task* tapaTask = nullptr;
pros::Task* wingsExpand = nullptr;
pros::Task* flywheelTask = nullptr;
pros::Task* liftTask = nullptr;
//bool climbState = false;
bool wing1Expand = false;
bool wing2Expand = false;
bool bothWingsExpand = false;
bool PTO_State = false;
bool PTO_StateD = false;
int buttonCountC = 0;
int buttonCountD = 0;
int liftGoal = 0;
bool climbOnC = false;
bool climbOnD = false;


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
#ifndef VARIABLES_h
#define VARIABLES_h

extern bool frontSlapaState;
extern bool initialSlapaMovement;
extern bool backSlapaState;
extern const double maxtapaShoot;
extern DrivePID Drive;
extern tapaSpeed tapaSpeedControl;
extern ArcTurn arcTurn;

extern double arcIntegral;
extern double arcDerivative;
extern double arcLastError;
extern double driveProportion;
extern double driveDerivative;
extern double driveLastError;
extern double angleProportion;
extern double angleDerivative;
extern double angleLastError;

extern double turnIntegral; 
extern double turnDerivative;
extern double turnLastError;

extern pros::Task* tapaTask;
extern pros::Task* wingsExpand;
extern bool wing1Expand;
extern bool wing2Expand;
extern bool bothWingsExpand;

extern bool reset;
extern bool flywheelOn;
extern int targetVoltage;
extern pros::Task* flywheelTask;

extern bool PTO_State;
extern pros::Task* liftTask;
extern int buttonCountC;
extern int buttonCountD;
extern int liftGoal;
extern  bool PTO_StateD;
extern bool climbOnC;
extern bool climbOnD;


#endif
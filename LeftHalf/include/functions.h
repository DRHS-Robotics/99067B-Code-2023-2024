#ifndef FUNCTIONS_h
#define FUNCTIONS_h

int sgn(double num);
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

#endif
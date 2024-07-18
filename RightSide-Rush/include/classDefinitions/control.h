#ifndef CONTROL_h
#define CONTROL_h

class DrivePID {
	public:
		double classPastCurrent;
		double calculatePower(double, double, double, double);
		double calculateAngle(double, double, double, double);
		void control_drive(double, double, double);
		void control_drive_back(double, double, double);
		void set(double pastCurr);

 };

class TurnPID {
	public:
        double calculateError(double, double);
		double calculatePower(double, double, double, double, double);
        bool calculate(double, double);
 };

class ArcTurn{
	public:
		double currPos;
		double calculatePower(double, double, double, double, double, double);
		double calculateTurnSide(double, double, double, double, bool);
    	bool calculate(double, double);
		void FArcTurn(double, double, double, double);
		void BArcTurn(double, double, double, double);
		void set(double curr);
 };


#endif
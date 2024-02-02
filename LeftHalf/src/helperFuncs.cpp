#include "main.h"

int sgn(double num){
    if(num >= 0){
        return 1;
    }else{
        return -1;
    }
}

double returnThetaInRange(double thetaAngle){
    double output = 0.0;
    if(thetaAngle >= 360.0){
        output = fmod(thetaAngle, 360.0);
    }
    if(thetaAngle < 0.0){
        output = fmod(thetaAngle, 360.0) + 360.0;
    }
    if(thetaAngle >= 0 && thetaAngle < 360.0){
        output = thetaAngle;
    }

    return output;
}

void tapaSpeed::tapaSet(int tapaMatch, int tapaSingle){
	this -> tapaSingleShot = tapaSingle;
	this -> tapaMatchLoad = tapaMatch;
}

void ArcTurn::set(double curr){
	this -> currPos = curr;
}

void drive(double left, double right){
	ptoL_drive.move(left);
	tl_drive.move(left);
	bl_drive.move(left);
	ptoR_drive.move(right);
	tr_drive.move(right);
	br_drive.move(right);
}

void PTO_Drive(double left, double right){
	tl_drive.move(left);
	bl_drive.move(left);
	tr_drive.move(right);
	br_drive.move(right);
}

void resetPos(){
    ptoL_drive.set_zero_position(0);
    ptoR_drive.set_zero_position(0);
    bl_drive.set_zero_position(0);
    br_drive.set_zero_position(0);
    tl_drive.set_zero_position(0);
    tr_drive.set_zero_position(0);
}

double position(){
	return ((ptoL_drive.get_position() + ptoR_drive.get_position() + bl_drive.get_position() + br_drive.get_position() + tl_drive.get_position() + tr_drive.get_position())/6);
}

double angle(){
    return((imu1.get_rotation() + imu2.get_rotation())/2);
}

void control_flywheel_fn(){
	if(flywheelTask == nullptr){
		flywheelTask = new pros::Task{[=]{
		
		double rpm = 0;
		double rpmTarget = 0;
		double rpmError = 0;
		double rpmPastError = 0;
		double proportion = 0;
		double integral = 0;
		double derivative = 0;
		double motorVoltage = 0;
		//Variables to track values related to
		//RPM and voltage error

		//Tuneable constants to reduce osciallation
		//of precision-state flywheel PID
		double kP = 1;
		double kI = 0.08264748923;
		double kD = 0.006; //Conversion from RPM to voltage
		double kV = (10/3);
		double threshold = 140;
		double startkI = 100;
		int time_delay = 20;



		while(true){
			if(flywheelOn){ //If flywheel is set to be on
				rpm = fabs((flywheel.get_actual_velocity())*6);
				rpmTarget = double (targetVoltage/kV);
				rpmError = rpmTarget - rpm;
				//Obtain RPM error
				proportion = rpmError;
				//Proportion value is proportional to error
				if(kI != 0){
					//Integral anti-windup
					if(fabs(rpmError) < startkI){
						integral += rpmError; //Integral accumulation
					}
					if(sgn(rpmError) != sgn(rpmPastError)){
						integral = 0;
					}
				}

				derivative = ((rpmError - rpmPastError));
				//Activate derivative for slowing effect of flywheel

				//Range to apply bang-bang control to flywheel
				//Provides fastest recovery for flyhweel speed drop.
				if(rpmError > threshold){
					motorVoltage = 12000;
				}else{
					//PID used for max precision of flywheel.
					motorVoltage = (kV * rpmTarget) + (proportion*kP) + (integral*kI) + (derivative*kD);

				}

				//Clamps flywheel to prevent values out of range from
				//Being output to flywheel
				if(motorVoltage > 12000){
					motorVoltage = 12000;
				}else if(motorVoltage < 0){
					motorVoltage = 0;
				}

				rpmPastError = rpmError;
				//Past error for derivative.
				std::cout << "Motor Voltage " << motorVoltage << std::endl;
				std::cout << "Proporional " << proportion << std::endl;
				std::cout << "Integral : " << integral << std::endl; 
				std::cout << "Derivative " << derivative << std::endl;
				std::cout << "RPM : " << rpm << std::endl;
				std::cout << "RPM TARGET : " << rpmTarget << std::endl;
				flywheel.move_voltage(motorVoltage);
				//Move flywheel certain voltage based on error

				if(reset){
					//Reset values so old values are
					//not reused in another cycle
					proportion = 0;
					derivative = 0;
					integral = 0;
					rpmError = 0;
					rpmPastError = 0;
					rpmTarget = 0;
					rpm = 0;
					motorVoltage = 0;
				}

				pros::Task::delay(time_delay); //Delay for task so kernel does not starve processor of memory.
			}else{
				//If flywheel is set to be off, then stop flyhweel motor.
				flywheel.move(0);
			}
		}
		}};
	}
}

void lift_macro(){
	if(liftTask == nullptr){
		liftTask = new pros::Task{[=]{
			pros::Controller master(pros::E_CONTROLLER_MASTER);	
			int buttonCount = 0;
			const int liftGoal = 1000;
			int liftDis = (ptoL_drive.get_position() + ptoR_drive.get_position()) / 2;
			bool climbState = climbSwitch.get_value();
			double xVal = master.get_analog(ANALOG_LEFT_X);
			double yVal = master.get_analog(ANALOG_LEFT_Y);
			while(true){
			if(PTO_State){
				xVal = master.get_analog(ANALOG_LEFT_X);
				yVal = master.get_analog(ANALOG_LEFT_Y);
				PTO_Drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));
				if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
					if(fabs(yVal) > 20){
						PTO_State = true;
						pros::delay(200);
					}
					buttonCount++;
				}

				if(buttonCount == 1){
					if(liftDis < liftGoal){
						ptoL_drive.move(127);
						ptoR_drive.move(127);
					}else{
						ptoL_drive.move(0);
						ptoR_drive.move(0);
					}
				}
				if(buttonCount == 2){
					climbRelease.set_value(true);
					pros::delay(200);
					if(!climbState){
						ptoL_drive.move(-127);
						ptoR_drive.move(-127);
					}else{
						ptoL_drive.move(0);
						ptoR_drive.move(0);
					}
				}
			}
			pros::Task::delay(20);
			}
		}};
	}
}


// void initializeTapaTask(){
// 	if(tapaTask == nullptr){
// 		//Lambda task(inline task defintion so that new function does not need to be created)
// 		tapaTask = new pros::Task{[=]{
// 			pros::Controller master(pros::E_CONTROLLER_MASTER);
// 			bool switchState = tapaSwitch.get_value();
// 			double tapaPos = tapa.get_position();
// 			const int time_delay = 20;
// 			//Max speed for tapa match loading and tapa shooting
// 			///////////////////////////////////////////////////
// 			int finalCount = 0;
// 			int countLimit = 0;
//             bool reset = false;
//             bool resetPos = false;
//             bool tapaStop = false;

// 			//Logic: 
// 			//Automatically retract the tapa to a primed position
// 			//Toggle the matchloading slapa(on or off)
// 			//Release the tapa to shoot a singular triball, then retract back to a primed position


// 			while(true){
// 				switchState = tapaSwitch.get_value(); //Boolean value from the limit switch at the bottom of tapa
// 				tapaPos = tapa.get_position();
// 				if((!backSlapaState) && (!frontSlapaState)){
// 					//Automatically return to a primed position
// 					if((!switchState) && (!reset)){
// 						tapa.move(tapaSpeedControl.tapaSingleShot);
// 					}else{
// 						//Count to ensure that the tapa is properly contacting the limit switch
// 						// finalCount++;
// 						//Reset tapa encoder position to use for single shot
// 						// resetPos = true;

// 						// if(tapaSpeedControl.tapaSingleShot > 110){
// 						// 	countLimit = 1;
// 						// }else if((tapaSpeedControl.tapaSingleShot < 110) && (tapaSpeedControl.tapaSingleShot > 100)){
// 						// 	countLimit = 2;
// 						// }else{
// 						// 	countLimit = 7;
// 						// }
// 						tapa.move(0);
// 						tapa.set_zero_position(0);
// 						if(tapa.get_actual_velocity() > 65){
// 							reset = false;
// 						}else{
// 							reset = true;
// 						}
//                         tapaStop = false;
// 					}
// 				}else if((backSlapaState) && (!frontSlapaState)){
// 					//Reset final count so that tapa returns to position properly
// 					reset = false;
// 					finalCount = 0;
// 					if(backSlapaState){
// 						tapa.move(tapaSpeedControl.tapaMatchLoad);
// 					}
// 				}
// 				else if((!backSlapaState) && (frontSlapaState)){
// 					finalCount = 0;
// 					reset = false;
// 					if((tapaPos <= maxtapaShoot) && (!tapaStop)){
// 						tapa.move(tapaSpeedControl.tapaSingleShot);
// 					}else{
//                         tapa.move(0);
//                         tapaStop = true;
//                     }
// 				}
// 				else{
// 					tapa.move(0);
// 				}

// 				// if(resetPos){
// 				// 	tapa.set_zero_position(0);
// 				// 	resetPos = false;
// 				// }

// 				pros::Task::delay(time_delay);
// 			}
// 		}};
// 	}
// }

void initializeTapaTask(){
	if(tapaTask == nullptr){
		//Lambda task(inline task defintion so that new function does not need to be created)
		tapaTask = new pros::Task{[=]{
			pros::Controller master(pros::E_CONTROLLER_MASTER);
			bool switchState = tapaSwitch.get_value();
			double tapaPos = tapa.get_position();
			const int time_delay = 20;
			//Max speed for tapa match loading and tapa shooting
			///////////////////////////////////////////////////
			int finalCount = 0;
			int countLimit = 0;
            bool reset = false;
            bool resetPos = false;
            bool tapaStop = false;

			//Logic: 
			//Automatically retract the tapa to a primed position
			//Toggle the matchloading slapa(on or off)
			//Release the tapa to shoot a singular triball, then retract back to a primed position


			while(true){
				switchState = tapaSwitch.get_value(); //Boolean value from the limit switch at the bottom of tapa
				if(frontSlapaState){
					slapper.move(127);
				}else{
					slapper.move(0);
				}

				pros::Task::delay(time_delay);
			}
		}};
	}
}


void setWings(){
	if(wingsExpand == nullptr){
		wingsExpand = new pros::Task{[=]{
			while(true){
				if(wing1Expand){
					wings1.set_value(true);
				}else{
					wings1.set_value(false);
				}

				if(wing2Expand){
					wings2.set_value(true);
				}else{
					wings2.set_value(false);
				}

				if(bothWingsExpand){
					wings1.set_value(true);
					wings2.set_value(true);
				}else{
					wings1.set_value(false);
					wings2.set_value(false);
				}
				pros::Task::delay(20);
			}
		}};
	}
}

double ArcTurn::calculatePower(double radius, double error, double maxOppPower, double kP, double kI, double kD){
	double proportion;
	double oppPower;

	proportion = error * kP;
	arcIntegral = arcIntegral + (error*kI);
	std::cout << "Arc Integral : " << arcIntegral << std::endl;

	arcDerivative = (error - arcLastError) * kD;
	oppPower = proportion + arcIntegral + arcDerivative;

	arcLastError = error;

	if(fabs(oppPower) > maxOppPower){
		oppPower = sgn(oppPower) * maxOppPower;
	}

	return oppPower;
}

double ArcTurn::calculateTurnSide(double target, double error, double startError, double radius, bool left){
	const double pi = 2*acos(0);
	double targetRads = target*(pi/180);
	double arcLengthLeft = 0;
	double arcLengthRight = 0;
	double powerRatio = 0;
	const double robotWidth = 13.5; //Changed from 15 inches
	double turnSidePower;

	if(left){
		arcLengthLeft = radius * targetRads;
		arcLengthRight = (radius + robotWidth) * targetRads;
		powerRatio = arcLengthRight/arcLengthLeft;
	}else{
		arcLengthRight = radius * targetRads;
		arcLengthLeft = (radius + robotWidth) * targetRads;
		powerRatio = arcLengthLeft/arcLengthRight;
	}

	turnSidePower = radius*powerRatio;

	if(error <= (startError/10)){
		turnSidePower = (turnSidePower/4);
	}else if(error <= (startError/5)){
		turnSidePower = (turnSidePower/3);
	}else if(error <= (startError/3)){
		turnSidePower = (turnSidePower/2);
	}else{
		turnSidePower = turnSidePower;
	}

	return turnSidePower;

}

bool ArcTurn::calculate(double target, double angle){
		double lTurnVal, rTurnVal = 0;
		bool left = false;
		if(target > angle){
			lTurnVal = fabs(angle + fabs(360 - target));
			rTurnVal = fabs(target - angle);
		} else{
			lTurnVal = fabs(angle - target);
			rTurnVal = fabs(target + fabs(360 - angle));
		}

		if(lTurnVal > rTurnVal){
			left = false;
		}else if(rTurnVal > lTurnVal){
			left = true;
		}

		return left;
}

double TurnPID::calculatePower(double error, double maxPower, double kP, double kI, double kD){
	double proportion;
	double power;

	proportion = error * kP;
	turnIntegral = turnIntegral + (error*kI);
	std::cout << "Integral " << turnIntegral << std::endl;
	// if(integral > integralMax){
	// 	integral = integralMax;
	// }
	turnDerivative = (error - turnLastError) * kD;
	std::cout << "Derivative " << turnDerivative << std::endl;
	power = proportion + turnIntegral + turnDerivative;

	turnLastError = error;

	if(fabs(power) > maxPower){
		power = sgn(power) * maxPower;
	}

	// if(power < 24){
	// 	power = 24;
	// }

	return power;
}

bool TurnPID::calculate(double target, double angle){
	double lTurnVal, rTurnVal = 0;
	bool left = false;
	if(target > angle){
		lTurnVal = fabs(angle + fabs(360 - target));
		rTurnVal = fabs(target - angle);
	} else{
		lTurnVal = fabs(angle - target);
		rTurnVal = fabs(target + fabs(360 - angle));
	}

	if(lTurnVal > rTurnVal){
		left = false;
	}else if(rTurnVal > lTurnVal){
		left = true;
	}else if(lTurnVal == rTurnVal){
		left = false;
	}

	return left;
}

double DrivePID::calculatePower(double error, double maxPower, double kP, double kD){
	double power = 0;

	driveProportion = error * kP;
	driveDerivative = (error - driveLastError) * kD;
	power = driveProportion + driveDerivative;

	driveLastError = error;

	if(fabs(power) > maxPower){
		power = sgn(power) * maxPower;
	}

	// if(power < 24){
	// 	power = 24;
	// }

	return power;
}

double DrivePID::calculateAngle(double error, double kP, double kD, double angleMaxPower){
	double power;

	angleProportion = error * kP;
	angleDerivative = (error - angleLastError) * kD;
	power = angleProportion + angleDerivative;

	angleLastError = error;

	if(fabs(power) > angleMaxPower){
		power = sgn(power) * angleMaxPower;
	}

	return power;
}

void ArcTurn::FArcTurn(double target, double radius, double maxPower, double arckI){
	currPos = 0;
	double currentActualAngle = angle();
	double currentPos = position();
	double pastCurrent = 0.0;
	double currentLeftPower = 0;
	double currentRightPower = 0;
	float arckP = 1; //Changed from 1.9
	float arckD = 1;
	double error = target-currentActualAngle;
	const double startError = error;
	int count = 0;
	int exitCount = 0;

	bool left = false;


	while((!(((currentActualAngle<=(target+1))) && (currentActualAngle>=(target-1))))){
		currentActualAngle = returnThetaInRange(angle());
		std::cout << "Current angle: " << currentActualAngle << std::endl;
		currentPos = position();
		pastCurrent = this->currPos;
		this->currPos = currentPos;


		left = calculate(target, currentActualAngle);
		
		error = (target - currentActualAngle);

		if(error < 0){
			while(error < 0){
				error = 360 + error;
			}
		}else if(error > 360){
			while(error > 360){
				error = error - 360;
			}
		}
		std::cout << "Left Bool : " << left << std::endl;
		if(left){
			error = (360 - error);
			currentRightPower = calculatePower(radius, error, maxPower, arckP, arckI, arckD);
			currentLeftPower = calculateTurnSide(target, error, startError, radius, left);
			// std::cout << "Opposite Side Power: "<< currentRightPower << std::endl;
			// std::cout << "Turn Side Power: "<< currentLeftPower << std::endl;
			// std::cout << "Error : " << error << std::endl;
		}else if(!left){
			currentLeftPower = calculatePower(radius, error, maxPower, arckP, arckI, arckD);
			currentRightPower = calculateTurnSide(target, error, startError, radius, left);
			// std::cout << "Opposite Side Power: "<< currentLeftPower << std::endl;
			// std::cout << "Turn Side Power: "<< currentRightPower << std::endl;
			// std::cout << "Error : " << error << std::endl;
		}else{
			currentLeftPower, currentRightPower = 0;
		}

		if(((currentActualAngle<=(target+1))&&(currentActualAngle>=(target-1)))){
			count++;
		}

		if(count > 3){
			arcIntegral = 0;
			arcDerivative = 0;
			arcLastError = 0;
			break;
		}

		if(pastCurrent == currentPos){
			exitCount++;
		}else{
			exitCount = 0;
		}

		if(exitCount > 3){
			arcIntegral = 0;
			arcDerivative = 0;
			arcLastError = 0;
			break;
		}


		drive(currentLeftPower, currentRightPower);

		pros::delay(20);
	}
	drive(0,0);
}

void ArcTurn::BArcTurn(double target, double radius, double maxPower, double arckI){
	currPos = 0;
	double currentActualAngle = angle();
	double currentPos = position();
	double pastCurrent = 0.0;
	double currentLeftPower = 0;
	double currentRightPower = 0;
	float arckP = 1; //Changed from 1.75
	float arckD = 1;
	double error = target-currentActualAngle;
	const double startError = error;
	int count = 0;
	int exitCount = 0;

	bool left = false;


	while((!(((currentActualAngle<=(target+1))) && (currentActualAngle>=(target-1))))){
		currentActualAngle = returnThetaInRange(angle());
		currentPos = position();
		pastCurrent = this->currPos;
		this->currPos = currentPos;

		left = calculate(target, currentActualAngle);
		
		error = (target - currentActualAngle);

		if(error < 0){
			while(error < 0){
				error = 360 + error;
			}
		}else if(error > 360){
			while(error > 360){
				error = error - 360;
			}
		}
		std::cout << "Left Bool : " << left << std::endl;
		if(left){
			error = (360 - error);
			currentLeftPower = calculatePower(radius, error, maxPower, arckP, arckI, arckD);
			currentRightPower = calculateTurnSide(target, error, startError, radius, left);
			// std::cout << "Opposite Side Power: "<< currentRightPower << std::endl;
			// std::cout << "Turn Side Power: "<< currentLeftPower << std::endl;
			// std::cout << "Error : " << error << std::endl;
		}else if(!left){
			currentRightPower = calculatePower(radius, error, maxPower, arckP, arckI, arckD);
			currentLeftPower = calculateTurnSide(target, error, startError, radius, left);
			// std::cout << "Opposite Side Power: "<< currentLeftPower << std::endl;
			// std::cout << "Turn Side Power: "<< currentRightPower << std::endl;
			// std::cout << "Error : " << error << std::endl;
		}else{
			currentLeftPower, currentRightPower = 0;
		}

		if(((currentActualAngle<=(target+1))&&(currentActualAngle>=(target-1)))){
			count++;
		}

		if(count > 0){
			arcIntegral = 0;
			arcDerivative = 0;
			arcLastError = 0;
			break;
		}

		if(pastCurrent == currentPos){
			exitCount++;
		}else{
			exitCount = 0;
		}

		if(exitCount > 3){
			arcIntegral = 0;
			arcDerivative = 0;
			arcLastError = 0;
			break;
		}


		drive(currentLeftPower*-1, currentRightPower*-1);

		pros::delay(20);
	}
	drive(0,0);
}

void control_turn(double target, double maxPower, double turnkI){
	int count = 0;
	double currentActualAngle = angle();
	double turnPower = 0;
	double currentLeftPower = 0;
	double currentRightPower = 0;
	float turnkP = 1;
	float turnkD = 3;
	double error = target-currentActualAngle;
	TurnPID turnPID;

	bool left = false;


	while((!(((currentActualAngle<=(target+0.4))) && (currentActualAngle>=(target-0.4))))){
		currentActualAngle = returnThetaInRange(angle());
		std::cout << "Current angle: " << currentActualAngle << std::endl;


		left = turnPID.calculate(target, currentActualAngle);
		error = (target - currentActualAngle);

		if(error < 0){
			while(error < 0){
				error = 360 + error;
			}
		}else if(error > 360){
			while(error > 360){
				error = error - 360;
			}
		}
		std::cout << "Turn Error : " << error << std::endl;


		if(left){
			error = (360 - error);
			std::cout << "Turn Error : " << error << std::endl;
			turnPower = turnPID.calculatePower(error, maxPower, turnkP, turnkI, turnkD);
		}else if(!left){
			turnPower = turnPID.calculatePower(error, maxPower, turnkP, turnkI, turnkD);
		}else{
			turnPower = 0;
		}

		// std::cout << "Turn " << turnPower << endl;
		// std::cout << "Angle " << currentActualAngle << endl;
		// std::cout << "Error " << error << endl;
		// std::cout << "Count " << count << endl;
		// std::cout << "Left " << left << endl;

		if(left){
			currentLeftPower = turnPower*-1;
			currentRightPower = turnPower;
		}else{
			currentLeftPower = turnPower;
			currentRightPower = turnPower*-1;
		}
		if(((currentActualAngle<=(target+0.4))&&(currentActualAngle>=(target-0.4)))){
			turnIntegral = 0;
			turnDerivative = 0;
			turnLastError = 0;
			break;
		}

		drive(currentLeftPower, currentRightPower);

		pros::delay(20);
	}
	drive(0,0);
}

void DrivePID::set(double pastCurr){
	this -> classPastCurrent = pastCurr;
}

void DrivePID::control_drive(double target, double maxPower, double currentDesiredAngle){
	resetPos();
	classPastCurrent = 0;
	int count = 0;
	int exitCount = 0;
	double currentActualAngle = angle();
	double current = position();
	double pastCurrent = 0;
	double distancePower = 0;
	double anglePower = 0;
	double currentLeftPower = 0;
	double currentRightPower = 0;
	const float drivekP = 0.20;
	const float drivekD = 0.04;
	const float turnkP = 0.2; //Changed from 0.04
	const float turnkD = 0.15;
	double angleError = 0;
    const double angleMaxPower = 20.;
	DrivePID drivePID;

	while(!((current<=(target+17))&&(current>=(target-17))) || !(count>4)){
		current = position();
		pastCurrent = this->classPastCurrent;
		this->classPastCurrent = current;
		currentActualAngle = returnThetaInRange(angle());

		angleError = (currentDesiredAngle-currentActualAngle);
		if(angleError < 0){
			while(angleError < 0){
				angleError = 360 + angleError;
			}
		}else if(angleError > 360){
			while(angleError > 360){
				angleError = angleError - 360;
			}
		}

		distancePower = drivePID.calculatePower(target - current, maxPower, drivekP, drivekD);
		anglePower = drivePID.calculateAngle(angleError, turnkP, turnkD, angleMaxPower);

		currentLeftPower = distancePower - anglePower;
		currentRightPower = distancePower + anglePower;

		if(((current<=(target+17))&&(current>=(target-17)))){
			count++;
		}
		else{
			count = 0;
		}

		if(current >= (target+17)){
			driveProportion = 0;
			driveDerivative = 0;
			driveLastError = 0;
			break;
		}

		if(pastCurrent == current){
			exitCount++;
		}else{
			exitCount = 0;
		}

		if(exitCount > 3){
			driveProportion = 0;
			driveDerivative = 0;
			driveLastError = 0;
			break;
		}

		drive(currentLeftPower, currentRightPower);
		// std::cout << "Rotation Sensor Test : " << vertEncd.get_position() << endl;

		pros::delay(20);
	}
	drive(0,0);
}

void DrivePID::control_drive_back(double target, double maxPower, double currentDesiredAngle){
	resetPos();
	classPastCurrent = 0;
	int count = 0;
	int exitCount = 0;
	double currentActualAngle = angle();
	double current = position();
	double pastCurrent = 0;
	double distancePower = 0;
	double anglePower = 0;
	double currentLeftPower = 0;
	double currentRightPower = 0;
	const float drivekP = 0.21;
	const float drivekD = 0.04;
	const float turnkP = 0.2; //Changed from 0.04
	const float turnkD = 0.1;
	double angleError = 0;
    const double angleMaxPower = 20.;
	DrivePID drivePID;
	target = (fabs(target))*-1;
	//Problem came about from making maxPower negative.


	while((!((current<=(target+17))&&(current>=(target-17)))) || !(count>4)){
		current = position();
		pastCurrent = this ->classPastCurrent;
		this->classPastCurrent = current;
		currentActualAngle = returnThetaInRange(angle());

		angleError = (currentDesiredAngle-currentActualAngle);
		if(angleError < 0){
			while(angleError < 0){
				angleError = 360 + angleError;
			}
		}else if(angleError > 360){
			while(angleError > 360){
				angleError = angleError - 360;
			}
		}

		distancePower = drivePID.calculatePower(target - current, maxPower, drivekP, drivekD);
		anglePower = drivePID.calculateAngle(angleError, turnkP, turnkD, angleMaxPower);

		currentLeftPower = distancePower - anglePower;
		currentRightPower = distancePower + anglePower;

		currentLeftPower = currentLeftPower;
		currentRightPower = currentRightPower;

		if(((current<=(target+17))&&(current>=(target-17)))){
			count++;
		}
		else{
			count = 0;
		}

		drive(currentLeftPower, currentRightPower);

		if(pastCurrent == current){
			exitCount++;
		}else{
			pastCurrent = current;
			exitCount = 0;
		}

		if(exitCount > 5){
			driveProportion = 0;
			driveDerivative = 0;
			driveLastError = 0;
			break;
		}

		if(current <= (target-17)){
			driveProportion = 0;
			driveDerivative = 0;
			driveLastError = 0;
			break;
		}

		pros::delay(20);
	}
	drive(0,0);
}

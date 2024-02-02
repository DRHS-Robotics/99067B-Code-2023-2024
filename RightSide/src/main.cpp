#include "main.h"
using namespace std;

// DRIVE MOTORS
//port 17 bad

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(pros::on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	////////////////////////////////////////////////////////////////////////////
	//First thing:
	//Make sure all of the motor ports match up with the motor ports in the globals.cpp file.
	////////////////////////////////////////////////////////////////////////////
	// setWings();
	//Leave these lines of code in here
	//These make sure the classes in the control.h file work
	Drive.set(0);
	arcTurn.set(0);
	intake2.move(-127);
	pros::delay(100);
	intake2.move(127);
	Drive.control_drive(300, 110, 0);
	pros::delay(400);
	//control_turn(355, 110, 0.45);
	Drive.control_drive_back(1350, 70, 0);
	arcTurn.BArcTurn(278, 55, 100, 0.015);
	Drive.control_drive_back(550, 127, 278);
	Drive.control_drive(400, 127, 278);
	//wing2Expand = true;
	//wings2.set_value(true);
	//control_turn(270, 90, 0.016);
	//pros::delay(250);
	//wings2.set_value(false);
	//control_turn(127, 100, 0.012);
	//intake2.move(-127);
	//Drive.control_drive(825, 120, 127);
	control_turn(93, 100, 0.006);
	intake2.move(-127);
	Drive.control_drive(700, 127, 93);
	Drive.control_drive_back(450, 127, 93);
	intake2.move(0);
	control_turn(26, 100, 0.026);
	intake2.move(127);
	pros::delay(75);
	Drive.control_drive(2000, 80, 26);
	pros::delay(75);
	control_turn(160, 110, 0.0081);
	intake2.move(-127);
	Drive.control_drive(1400, 100, 160);
	arcTurn.BArcTurn(61, 1, 100, 0.009);
	pros::delay(100);
	intake2.move(127);
	Drive.control_drive(900, 90, 61);
	control_turn(180, 100, 0.01);
	intake2.move(-127);
	Drive.control_drive(1400, 127, 180);
	Drive.control_drive_back(700, 127, 180);
	//wings2.set_value(false);
	intake2.move(0);
	
	//This is how to activate the pneumatics
	//Set the value to false to deactivate the pneumatics.
	//wings1.set_value(true);
	//wings2.set_value(true);
	//climbRelease.set_value(true);
	//climbRelease.set_value(false);

	//This is how to move the intake, tapa, or any specific motors.
	//Change the value to negative to move the motors backwards.
	//Change the value to zero to stop the motors.
	//intake.move(127);
	//tapa.move(127);
	//intake.move(0);
	//tapa.move(0);

	//To move the motors for a specifc amount of time, use this:
	//pros::delay(time in milliseconds)

	//You don't always need to stop the motors to change their direction
	//For example:
	//intake.move(127);
	//control_drive(1000, 110, 10);
	//intake.move(-127);
	//Move the intake forward while driving forwards
	//Move the intake backwards after driving forwards

	//Speaking of the control drive function:
	//Structure of the control drive function:
	//Drive.control_drive(target in encoder ticks to drive forward, max speed, angle you want to follow)
	//Drive.control_drive_back(target in encoder ticks to drive backwards, max speed, angle you want to follow)
	//Max speed limited to -127 and +127.
	//These should work fine. If they aren't, then try lowering the speed if going too fast or
	//Or increase the distance if not going far enough.

	//Turns:
	//control_turn(target angle, maxSpeed, kI constant);
	//Target angle is the angle you want to turn towards(limited from 0 to 360 degrees)
	//Max speed is limited from -127 and +127.
	//kI constant is something that is really finicky. The only way to get the perfect kI value is tune
	//Start out with 0.006 for the kI constant. If the robot is turning too slow, then increase the speed
		//You will know the kI value needs to be increased if the robot is slow to reach the target
	//If the robot is fighting itself, then decrease the kI value
		//You will know the kI value needs to be decreased if the robot is fighting itself.

	//ArcTurns:
	//Front arc turn
	//arcTurn.FArcTurn(target angle, radius of circle, maxSpeed, kI).
	//Back arc turn
	//arcTurn.BArcTurn(target angle, radius of circle, maxSpeed, kI).
	//Once again, start the kI value at 0.006. Tune as necessary
	//Smaller radius means a smaller arc turn
	//Bigger radius means a larger arc turn

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	initializeTapaTask();
	// control_flywheel_fn();
	// int tapaPosition = tapa.get_position();
	bool speedControl = false;
	bool wingsState = false;
	bool leftWing = false;
	bool rightWing = false;
	bool climbState = false;
	bool matchLoadState = false;
	bool flywheelState = false;


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		(pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		(pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		
		double xVal = master.get_analog(ANALOG_LEFT_X);
		double yVal = master.get_analog(ANALOG_LEFT_Y);

		// tapaPosition = tapa.get_position();

			// PTO_Drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));
			// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			// 	ptoL_drive.move(127);
			// 	ptoR_drive.move(127);
			// }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			// 	ptoL_drive.move(-127);
			// 	ptoR_drive.move(-127);
			// }else{
			// 	ptoL_drive.move(0);
			// 	ptoR_drive.move(0);
			// }
		drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake2.move(127);
		}else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake2.move(-127);
		}else{
			intake2.move(0);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			// backSlapaState = !backSlapaState;
			// frontSlapaState = false;
			// flywheelState = !flywheelState;
			frontSlapaState = !frontSlapaState;
		}

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
		// 	frontSlapaState = !frontSlapaState;
		// 	backSlapaState = false;
		// }

		if(flywheelState){
			flywheelOn = true;
			reset = false;
		}else{
			flywheelOn = false;
			reset = true;
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			speedControl = !speedControl;
		}

		if(speedControl){
			// tapaSpeedControl.tapaSet(127, 127);
			targetVoltage = 8000;
		}else{
			// tapaSpeedControl.tapaSet(127, 127);
			targetVoltage = 7000;
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			wingsState = !wingsState;
			wings1.set_value(wingsState);
			wings2.set_value(wingsState);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			leftWing = !leftWing;
			wings2.set_value(leftWing);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			rightWing = !rightWing;
			wings1.set_value(rightWing);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			climbState = !climbState;
			climbRelease.set_value(climbState);
		}

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
		// 	matchLoadState = !matchLoadState;
		// 	matchLoad.set_value(matchLoadState);
		// }

		if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))){
			PTO_State = !PTO_State;
		}

//&& (fabs(yVal) > 0.))

		// pros::screen::print(pros::E_TEXT_MEDIUM, 3, "tapa POSITION : %3d", tapaPosition);

		pros::delay(20);
	}
}
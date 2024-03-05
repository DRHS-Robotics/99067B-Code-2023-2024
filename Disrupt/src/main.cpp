#include "main.h"
using namespace std;

// DRIVE MOTORS
//port 18 bad
//port 5 bad 

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
    //control_turn(angle, maxSpeed, turnkP);
	// Drive.set(0);
	// arcTurn.set(0);
    // intake2.move(127);
    // pros::delay(200);
    // intake2.move(0);    
    // wings2.set_value(true);
    // // Drive.control_drive(175, 127, 0);
    // control_turn(327, 127, 0.075, 2.5, true);
    // wings2.set_value(false);
    // control_turn(254, 90, 0.012, 2.8, false);
    // pros::delay(100);
    // intake2.move(-127);
    // Drive.control_drive(1800, 127, 254);
    // pros::delay(100);
    // Drive.control_drive_back(900, 110, 254);
    // arcTurn.BArcTurn(0, 6, 100, 0.006);
    // // control_turn(355, 115, 1.5);
    // intake2.move(127);
    // Drive.control_drive(2000, 105, 0);
    // Drive.control_drive(500, 80, 0);
    
    Drive.set(0);
	arcTurn.set(0);
    intake2.move(127);
    pros::delay(200);
    intake2.move(0);   
    frontWings1.set_value(true);
    // pros::delay(600);
    frontWings1.set_value(false);
    Drive.control_drive(1500, 127, 0);
    control_turn(85, 127, 0.015, 3.5, false);
    frontWings1.set_value(true);
    Drive.control_drive(900, 127, 85);
    // wings2.set_value(true);
    // control_turn(267, 115, 1.5);
    // control_turn(0, 115, 1.5);
    // wings2.set_value(false);
    // control_turn(0, 100, 0.015);
    // Drive.control_drive(1700, 100, 0);
    // control_turn(270,  100, 0.015);
    // Drive.control_drive(400, 70, 0);



	// Drive.control_drive(100, 40, 0);
	// arcTurn.BArcTurn(90, 15, 100, 0.009);
    // intake2.move(0);
	// Drive.control_drive_back(450, 120, 90);
	// Drive.control_drive(300, 100, 90);
	// control_turn(60, 110, 0.039);
	// Drive.control_drive(700, 100, 60);
	// wings2.set_value(true);
	// control_turn(340, 120, 0.045);
    // wings2.set_value(false);
	// control_turn(200, 40, 0.0035);
	// // Drive.control_drive_back(300, 100, 250);
	// Drive.control_drive_back(1750, 80, 200);
	// matchLoad.set_value(true);
	
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
	//control_turn(target angle, maxSpeed, kP constant);
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
    // initializeTapaTask();
    lift_macro();
    // control_flywheel_fn();
    // int tapaPosition = tapa.get_position();
    bool speedControl = false;
    bool wingsState = false;
    bool leftWing = false;
    bool rightWing = false;
    bool matchLoadState = false;
    bool flywheelState = false;
    bool ratchetState = false;
    int count = 0;
    // buttonCountC = 0;
    // buttonCountD = 0;


    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
        (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
        (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        // initialSlapaMovement = true;
        
        double xVal = master.get_analog(ANALOG_LEFT_X);
        double yVal = master.get_analog(ANALOG_LEFT_Y);

        // cout << "xVal: " << xVal << endl;
        // cout << "yVal: " << yVal << endl;

        // if(count < 5){
        //     climbRelease.set_value(true);
        // }

        // tapaPosition = tapa.get_position();
        drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));


            // PTO_Drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));
            // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            //  ptoL_drive.move(127);
            //  ptoR_drive.move(127);
            // }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            //  ptoL_drive.move(-127);
            //  ptoR_drive.move(-127);
            // }else{
            //  ptoL_drive.move(0);
            //  ptoR_drive.move(0);
            // }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake2.move(127);
        }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake2.move(-127);
        }else{
            intake2.move(0);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            climb1.move_velocity(100);
            climb2.move_velocity(200);
        }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            climb1.move_velocity(-100);
            climb2.move_velocity(-200);
        }else{
            climb1.move_velocity(0);
            climb2.move_velocity(0);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            frontSlapaState = !frontSlapaState;
        }

        
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            backSlapaState = !backSlapaState;
        }
        // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
        //  frontSlapaState = !frontSlapaState;
        //  backSlapaState = false;
        // }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            wingsState = !wingsState;
            frontWings1.set_value(wingsState);
            frontWings2.set_value(wingsState);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            rightWing = !rightWing;
            wings2.set_value(rightWing);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            leftWing = !leftWing;
            wings1.set_value(leftWing);
        }

          if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            ratchetState = !ratchetState;
            climbRelease.set_value(ratchetState);
        }
        
        // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        //     climbState = !climbState;
        //     climbRelease.set_value(climbState);
        // }

        // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        //  matchLoadState = !matchLoadState;
        //  matchLoad.set_value(matchLoadState);
        // }

        // if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))){
        //     buttonCountC++;
        // }

        // if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))){
        //     buttonCountD++;
        // }

        // if(buttonCountC == 1){
        //     // ptoL_drive.set_zero_position(0);
        //     // ptoR_drive.set_zero_position(0);
        //     liftGoal = 5000;
        //     PTO_State = true;
        //     // PTO.set_value(PTO_State);
        //     pros::delay(200);
        // }
        // if(buttonCountC == 2){
        //     climbOnC = true;
        //     climbRelease.set_value(climbOnC);
        //     pros::delay(200);
        // }

        //  if(buttonCountD == 1){
        //     // ptoL_drive.set_zero_position(0);
        //     // ptoR_drive.set_zero_position(0);
        //     PTO_StateD = true;
        //     // PTO.set_value(PTO_StateD);
        //     pros::delay(200);
        // }
        // if(buttonCountD == 2){
        //     climbOnD = true;
        //     climbRelease.set_value(climbOnD);
        //     pros::delay(200);
        // }

        // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        //     // climbOnD = false;
        //     // climbOnC = false;
        //     // buttonCountC = 0;
        //     // buttonCountD = 0;
        //     // PTO_State = false;
        //     // PTO_StateD = false;
        //     // PTO.set_value(false);
        //     climbRelease.set_value(false);
        // }

        // if((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))){
        //     climbRelease.set_value(true);
        // }
       
        // if(count < 10){
        //     count++;
        // }else{
        //     count = 11;
        // }

//&& (fabs(yVal) > 0.))

        // pros::screen::print(pros::E_TEXT_MEDIUM, 3, "tapa POSITION : %3d", tapaPosition);

        pros::delay(20);
    }
}
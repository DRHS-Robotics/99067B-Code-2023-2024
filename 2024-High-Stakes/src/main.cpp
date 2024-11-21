#include "main.h"
using namespace std;
using namespace pros;
using namespace lemlib;


enum Auto {redR, redL, blueR, blueL};
Auto auton = blueL;


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0);

    
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });


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
	if(auton == blueL){
		chassis.setPose(0, 0, 0);
		chassis.moveToPoint(0, -33, 1250, {.forwards = false, .maxSpeed = 115}, false);
		chassis.turnToHeading(38, 1000, {.maxSpeed = 100});
		chassis.moveToPoint(-6, -41.25, 1500, {.forwards = false, .maxSpeed = 50}, false);
		clip1.set_value(true);
		clip2.set_value(true);
		delay(300);
		robot.conveyor.move(127);
		robot.intake.move(127);
		chassis.turnToHeading(-24, 1000, {.maxSpeed = 100}, false);
		chassis.moveToPoint(-10, -33, 1000, {.maxSpeed = 100}, false);
		robot.conveyor.move(0);
		delay(675);
		robot.conveyor.move(127);
		delay(675);
		clip1.set_value(false);
		clip2.set_value(false);
		chassis.turnToHeading(95, 1000, {.maxSpeed = 90}, false);
		chassis.moveToPoint(-26.5, -27, 2000, {.forwards = false, .maxSpeed = 50}, false);
		clip1.set_value(true);
		clip2.set_value(true);
		delay(200);
		chassis.turnToHeading(-47, 1000, {.maxSpeed = 90}, false);
		chassis.moveToPoint(-46, -10, 2000, {.maxSpeed = 90}, false);
		stick.set_value(true);
		chassis.turnToHeading(-151, 1000, {.maxSpeed = 55}, false);
		stick.set_value(false);
		chassis.turnToHeading(-132, 1000, {.maxSpeed = 90}, false);
		chassis.moveToPoint(-57, -14, 1000, {.maxSpeed = 90}, false);
		chassis.turnToHeading(-175, 1000, {.maxSpeed = 90}, false);
		chassis.moveToPoint(-57, -20, 1000, {.maxSpeed = 90}, false);
	}
	

	// robot.conveyor.move(127);
	// delay(500);
	// chassis.turnToHeading(-18, 1000, {.maxSpeed = 100});
	// // robot.conveyor.move(0);
	// robot.intake.move(127);
	// // chassis.moveToPoint(-16, -35, 1500, {.maxSpeed = 115}, false);
	// // robot.conveyor.move(-127);
	// chassis.moveToPoint(-5, -23, 1500, {.maxSpeed = 70}, false);
	// // chassis.moveToPoint(-15, -30, 1500, {.maxSpeed = 110}, false);
	// delay(750);
	// clip1.set_value(false);
	// clip2.set_value(false);
	// chassis.turnToHeading(93, 1500, {.maxSpeed = 110}, false);
	// chassis.moveToPoint(-17, -23, 2000, {.forwards = false, .maxSpeed = 70}, false);
	// clip1.set_value(true);
	// clip2.set_value(true);
	// robot.intake.move(-127);
	// chassis.turnToHeading(-48, 1500, {.maxSpeed = 110}, false);
	// chassis.moveToPoint(-26, -11, 1500, {.maxSpeed = 100}, false);
	// stick.set_value(true);
	// // chassis.moveToPoint(-67, 5, 2000, {.maxSpeed =  90}, false);
	// robot.intake.move(127);
	// delay(500);
	// chassis.turnToHeading(-162, 1000, {.maxSpeed = 100}, false);
	// chassis.moveToPoint(-62, -25, 2000, {.maxSpeed = 70}, false);
	// robot.intake.move(0);
	// chassis.moveToPoint(-1, -27, 1000, {.forwards = false, .maxSpeed = 70}, false);
	// chassis.turnToHeading(90, 1000, {.maxSpeed = 100});
	// clip1.set_value(false);
	// clip2.set_value(false);
	// robot.conveyor.move(127);
	// delay(2000);
	// robot.conveyor.move(0);
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
	bool pistonState = false;
	bool stickState = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		double yVal = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		double xVal = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick

		robot.Drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));

		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
			pistonState = !pistonState;
		}

		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
			stickState = !stickState;
		}

		if(pistonState){
			clip1.set_value(true);
			clip2.set_value(true);
		}else{
			clip1.set_value(false);
            clip2.set_value(false);
		}

		if(stickState){
			stick.set_value(true);
		}else{
			stick.set_value(false);
		}
		
		if(master.get_digital(DIGITAL_R1)){
			robot.intake.move(127);
		}else if(master.get_digital(DIGITAL_R2)){
			robot.intake.move(-127);
		}else{
			robot.intake.move(0);
		}

		if(master.get_digital(DIGITAL_L1)){
			robot.conveyor.move(127);
		}else if(master.get_digital(DIGITAL_L2)){
			robot.conveyor.move(-127);
		}else{
			robot.conveyor.move(0);
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}
#include "main.h"
#include "lemlib/api.hpp"
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

void autonomous(){
    //Close Side
    // intake.move(127);
    // pros::delay(150);
    // intake.move(0);
    // wings1.set_value(true);
    // chassis.turnToHeading(-90, 2000, {.maxSpeed = 110}, false);
    // wings1.set_value(false);
    // chassis.turnToHeading(25.5, 1000, {.maxSpeed = 95}, false);
    // intake.move(127);
    // chassis.moveToPoint(13, 41, 1000, {.maxSpeed = 127}, false);
    // // Drive.control_drive_back(600, 127, 21);
    // chassis.moveToPoint(11.5, 36.5, 1000, {.maxSpeed = 100}, false);
    // chassis.turnToHeading(92, 1000, {.maxSpeed = 100}, false);
    // // frontWings1.set_value(true);
    // // frontWings1.set_value(true);
    // // Drive.control_drive(700, 127, 92);
    // chassis.moveToPoint(32, 38, 1000, {.maxSpeed = 90}, false);
    // // frontWings1.set_value(false);
    // // frontWings1.set_value(false);
    // // chassis.setPose(32, 38, 92);
    // // chassis.turnToHeading(44, 2000, {.maxSpeed = 30}, false);
    // // pros::delay(500);
    // chassis.moveToPoint(-6, -1, 2000, {.forwards = false, .maxSpeed = 70}, false);
    // chassis.turnToHeading(140, 1000, {.maxSpeed = 100}, false);
    // pros::delay(200);
    // intake.move(-127);
    // chassis.moveToPoint(-1, -10, 1000, {.maxSpeed = 100}, false);
    // chassis.turnToHeading(93, 1000, {.maxSpeed = 100}, false);
    // pros::delay(200);
    // chassis.moveToPoint(34, -12, 1000, {.maxSpeed = 70}, false);

    //Far Side(No Rush)
    // intake.move(127);
    // pros::delay(150);
    // intake.move(127);
    // chassis.moveToPoint(0, 5.8, 1000, {.maxSpeed = 100}, false);
    // chassis.moveToPoint(8, -34, 1000, {.forwards = false, .maxSpeed = 100}, false);
    // wings1.set_value(true);
    // chassis.moveToPoint(13.5, -40.3, 1000, {.forwards = false, .maxSpeed = 100}, false);
    // wings1.set_value(false);
    // chassis.moveToPoint(44, -55, 1000, {.forwards = false, .maxSpeed = 115}, false);
    // chassis.moveToPoint(18, -50, 1000, {.maxSpeed = 127}, false);
    // chassis.turnToHeading(90, 1000, {.maxSpeed = 100}, false);
    // intake.move(-127);
    // chassis.moveToPoint(38, -56.5, 1000, {.maxSpeed = 127}, false);
    // chassis.moveToPoint(20, -47, 1000, {.forwards = false, .maxSpeed = 100}, false);
    // chassis.turnToHeading(15.5, 1000, {.maxSpeed = 100}, false);
    // intake.move(127);
    // chassis.moveToPoint(34, -1, 1000, {.maxSpeed = 100}, false);
    // chassis.turnToHeading(158, 1000, {.maxSpeed = 100}, false);
    // intake.move(-127);
    // pros::delay(500);
    // chassis.turnToHeading(67, 1000, {.maxSpeed = 100}, false);
    // intake.move(127);
    // chassis.moveToPoint(59, 9, 1000, {.maxSpeed = 100}, false);
    // chassis.turnToHeading(180, 1000, {.maxSpeed = 100}, false);
    // frontWings1.set_value(true);
    // frontWings2.set_value(true);
    // intake.move(0);
    // chassis.moveToPoint(59, -24, 1000, {.maxSpeed = 127}, false);
    // chassis.moveToPoint(59, -12, 1000, {.forwards = false, .maxSpeed = 100}, false);
    // frontWings1.set_value(false);
    // frontWings2.set_value(false);
    
    //Rush Far Side
    intake.move(127);
    frontWings2.set_value(true);
    pros::delay(150);
    frontWings2.set_value(false);
    intake.move(0);
    intake.move(127);
    chassis.moveToPoint(-8.2, 46.4, 1000, {.maxSpeed = 127}, false);
    chassis.moveToPoint(0, -6, 1000, {.forwards = false, .maxSpeed = 110}, false);
    intake.move(-127);
    chassis.turnToHeading(58, 750, {.maxSpeed = 110}, false);
    //intake.move(-127);
    chassis.turnToHeading(-120, 750, {.maxSpeed = 115}, false);
    intake.move(0);
    chassis.moveToPoint(-35, -1, 1000, {.maxSpeed = 115}, false);
    intake.move(127);
    chassis.moveToPoint(4.8, 4.3, 1000, {.forwards = false, .maxSpeed = 115}, false);
    wings1.set_value(true);
    chassis.moveToPoint(9.5, 9.8, 750, {.forwards = false, .maxSpeed = 115}, false);
    wings1.set_value(false);
    chassis.moveToPoint(18, 32, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(18, 19.9, 1000, {.maxSpeed = 115}, false);
    chassis.turnToHeading(10, 750, {.maxSpeed = 110}, false);
    intake.move(-127);
    chassis.moveToPoint(24, 42, 1000, {.maxSpeed = 127}, false);
    chassis.moveToPoint(21, 20, 1000, {.forwards = false, .maxSpeed = 115}, false);
    chassis.turnToHeading(-433, 600, {.maxSpeed = 110}, false);
    intake.move(127);
    chassis.moveToPoint(-27, 46, 1000, {.maxSpeed = 115}, false);
    chassis.turnToHeading(53.7, 750, {.maxSpeed = 110}, false);
    intake.move(-127);
    chassis.moveToPoint(13, 49.5, 1000, {.maxSpeed = 115}, false);
    chassis.moveToPoint(-4.1, 33.4, 1000, {.forwards = false, .maxSpeed = 115}, false);
    chassis.moveToPoint(-21, 55, 1000, {.maxSpeed = 115}, false);
    chassis.turnToHeading(90, 750, {.maxSpeed = 110}, false);
    frontWings1.set_value(true);
    frontWings2.set_value(true);
    chassis.moveToPoint(10.5, 57.1, 1000, {.maxSpeed = 115}, false);
    chassis.moveToPoint(0, 55.2, 1000, {.forwards = false, .maxSpeed = 115}, false);
    frontWings1.set_value(false);
    frontWings2.set_value(false);




    //Old Rush Far Side
    // chassis.moveToPoint(8, -39, 1000, {.forwards = false, .maxSpeed = 127}, false);
    // wings2.set_value(true);
    // chassis.turnToHeading(120, 750, {.maxSpeed = 127}, false);
    // wings2.set_value(false);
    // intake.move(127);
    // chassis.moveToPoint(17.5, -46, 1000, {.maxSpeed = 127}, false);
    // chassis.moveToPoint(0.4, -38.3, 1000, {.forwards = false, .maxSpeed = 127}, false);
    // intake.move(0);
    // // frontWings1.set_value(true);
    // chassis.moveToPoint(-25, -44, 1000, {.maxSpeed = 127}, false);
    // // frontWings1.set_value(false);
    // chassis.moveToPoint(-14.7, -44, 1000, {.forwards = false, .maxSpeed = 127}, false);
    // chassis.turnToHeading(418.4, 750, {.maxSpeed = 110}, false);
    // intake.move(127);
    // chassis.moveToPoint(10.6, -30.8, 1000, {.maxSpeed = 127}, false);
    // chassis.turnToHeading(315, 750, {.maxSpeed = 110}, false);
    // chassis.moveToPoint(-15, -2, 1000, {.maxSpeed = 127}, false);
    // chassis.turnToHeading(225, 750, {.maxSpeed = 127}, false);
    // intake.move(-127);
    // pros::delay(300);
    // intake.move(0);
    // chassis.turnToHeading(40, 750, {.maxSpeed = 127}, false);
    // intake.move(127);
    // chassis.moveToPoint(18.9, 10.2, 1000, {.maxSpeed = 127}, false);
    // chassis.moveToPoint(-21.2, 3.2, 1000, {.forwards = false, .maxSpeed = 127}, false);
    // wings1.set_value(true);
    // chassis.moveToPoint(-25.1, -1.1, 1000, {.forwards = false, .maxSpeed = 127}, false);
    // wings1.set_value(false);
    // chassis.moveToPoint(-33, -21.1, 1000, {.forwards = false, .maxSpeed = 127}, false);
    // chassis.moveToPoint(-30.5, -10.6, 1000, {.maxSpeed = 127}, false);
    // chassis.turnToHeading(543, 750, {.maxSpeed = 127}, false);
    // intake.move(0);
    // chassis.moveToPoint(-29.5, -25.4, 1000, {.maxSpeed = 127}, false);
    // chassis.moveToPoint(-28.5, -13.1, 1000, {.forwards = false, .maxSpeed = 127}, false);

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
    lift_macro();
    bool speedControl = false;
    bool wingsState = false;
    bool leftWing = false;
    bool rightWing = false;
    bool matchLoadState = false;
    bool flywheelState = false;
    int count = 0;

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
        (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
        (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        // initialSlapaMovement = true;
        
        double xVal = master.get_analog(ANALOG_LEFT_X);
        double yVal = master.get_analog(ANALOG_LEFT_Y);

        drive((pow((yVal+xVal)/100,3)*100), (pow((yVal-xVal)/100,3)*100));

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move(127);
        }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(-127);
        }else{
            intake.move(0);
        }

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

        pros::delay(20);
    }
}

#include "main.h"

Robot robot({-3,-11,-12}, {4,13, 14}, {-2, 8}, -6, 5, MotorGears::green, MotorGears::blue);

//Adjust the distances later
// TrackingWheel horizontal_tracking_wheel(&robot.horizontalRot, Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
// TrackingWheel vertical_tracking_wheel(&robot.verticalRot, Omniwheel::NEW_275, -2.5);

pros::Controller master(pros::E_CONTROLLER_MASTER);
adi::DigitalOut clip1('A');
adi::DigitalOut clip2('B');
adi::DigitalOut stick('C');

pros::Task* intakeTask = nullptr;
void intakeMacro();
bool intakeState = false;

Drivetrain drivetrain(&robot.leftMotors, // left motor group
                              &robot.rightMotors, // right motor group
                              10, // 10 inch track width
                              Omniwheel::NEW_325, // using new 2.75" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &robot.imu1 // inertial sensor
);

// lateral PID controller
ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void intakeMacro(){
    if(intakeTask == nullptr){
        if(intakeTask == nullptr){
        intakeTask = new pros::Task{[=]{
        

        while(true){
            if((intakeState)){
				// std::cout << "Position" << position() << std::endl;
                if(robot.intake.get_actual_velocity() == 0){
                    robot.intake.move(-127);
                }else{
                    robot.intake.move(127);
                }
            }

            pros::Task::delay(20);
            }
        }};
    }
    }
}
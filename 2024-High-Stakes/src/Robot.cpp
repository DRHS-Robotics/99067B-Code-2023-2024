#include "main.h"
using namespace std;
using namespace pros;
using namespace v5;
using namespace lemlib;

class Robot{
    public:
        MotorGroup leftMotors;
        MotorGroup rightMotors;
        Motor intake;
        Motor conveyor;
        IMU imu1;
        // Rotation verticalRot;
        // Rotation horizontalRot;

        Robot(std::initializer_list<std::int8_t> leftMotors, std::initializer_list<std::int8_t> rightMotors, int intake, int conveyor, int imu1,  MotorGears greenGear, MotorGears blueGear) 
        : leftMotors(leftMotors, blueGear), rightMotors(rightMotors, blueGear), intake(intake, greenGear), conveyor(conveyor, blueGear), imu1(imu1) {
            // this->verticalRot.set_reversed(true);
            // this->horizontalRot.set_reversed(false);
        }

        void Drive(double leftPow, double rightPow){
            leftMotors.move(leftPow);
            rightMotors.move(rightPow);
        }
        void rightAuto(){
            chassis.setPose(0, 0, 0);
        }

        void leftAuto(){

        }

};


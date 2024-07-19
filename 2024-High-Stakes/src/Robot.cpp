#include "main.h"
using namespace std;
using namespace pros;
using namespace v5;

class Robot{
    public:
        MotorGroup leftMotors;
        MotorGroup rightMotors;
        Motor intake;
        IMU imu1;
        Rotation verticalRot;
        Rotation horizontalRot;

        Robot(std::initializer_list<std::int8_t> leftMotors, std::initializer_list<std::int8_t> rightMotors, int intake, int imu1, int verticalRot, int horizontalRot,  MotorGears greenGear, MotorGears blueGear) 
        : leftMotors(leftMotors, blueGear), rightMotors(rightMotors, blueGear), intake(intake, greenGear), imu1(imu1), verticalRot(verticalRot), horizontalRot(horizontalRot){
            this->verticalRot.set_reversed(true);
            this->horizontalRot.set_reversed(false);
        }

        void Drive(double leftPow, double rightPow){
            leftMotors.move(leftPow);
            rightMotors.move(rightPow);
        }
        void rightAuto(){

        }

        void leftAuto(){

        }

};


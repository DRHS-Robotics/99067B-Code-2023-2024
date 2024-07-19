#include "main.h"
using namespace std;
using namespace pros;

class Robot{
    private:
        MotorGroup leftMotors;
        MotorGroup rightMotors;
        Motor intake;
    
    public:
        Robot(std::initializer_list<std::int8_t> leftMotors, std::initializer_list<std::int8_t> rightMotors, int intake) : leftMotors(leftMotors), rightMotors(rightMotors), intake(intake){

        }
        void Drive(double leftPow, double rightPow);
        void standardAuto();

};
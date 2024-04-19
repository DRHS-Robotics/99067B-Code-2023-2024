#include "main.h"
#include "lemlib/api.hpp"

int sgn(double num){
    if(num >= 0){
        return 1;
    }else{
        return -1;
    }
}

void drive(double left, double right){
	fl_drive.move(left);
    fr_drive.move(right);
    bl_drive.move(left);
    br_drive.move(right);
	ml_drive.move(left);
	mr_drive.move(right);
    tl_drive.move(left);
    tr_drive.move(right);
}

void resetPos(){
    fl_drive.set_zero_position(0);
    fr_drive.set_zero_position(0);
    bl_drive.set_zero_position(0);
    br_drive.set_zero_position(0);
	ml_drive.set_zero_position(0);
	mr_drive.set_zero_position(0);
    tl_drive.set_zero_position(0);
    tr_drive.set_zero_position(0);
}

double position(){
	return ((fl_drive.get_position() + fr_drive.get_position() + bl_drive.get_position() + br_drive.get_position() + tl_drive.get_position() + tr_drive.get_position() + ml_drive.get_position() + mr_drive.get_position())/8);
}

double velocity(){
	return ((fl_drive.get_actual_velocity() + fr_drive.get_actual_velocity() + bl_drive.get_actual_velocity() + br_drive.get_actual_velocity() + tl_drive.get_actual_velocity() + tr_drive.get_actual_velocity() + ml_drive.get_actual_velocity() + mr_drive.get_actual_velocity())/8);
}

void lift_macro(){
//     if(liftTask == nullptr){
//         liftTask = new pros::Task{[=]{
        
//         while(true){
//             if(PTO_State){
//                 if(velocity() > 5){
//                     fl_drive.move_velocity(600);
//                     ml_drive.move_velocity(600);
//                     tl_drive.move_velocity(200);
//                     bl_drive.move_velocity(600);
// 			        fr_drive.move_velocity(600);
// 			        mr_drive.move_velocity(600);
// 			        tr_drive.move_velocity(200);
// 			        br_drive.move_velocity(600);
//                 }else{
//                     fl_drive.move_velocity(0);
//                     ml_drive.move_velocity(0);
//                     tl_drive.move_velocity(0);
//                     bl_drive.move_velocity(0);
// 			        fr_drive.move_velocity(0);
// 			        mr_drive.move_velocity(0);
// 			        tr_drive.move_velocity(0);
// 			        br_drive.move_velocity(0);
//                 }
//             }

//             pros::Task::delay(20);
//             }
//         }};
//     }
}

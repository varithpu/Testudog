#include "../include/testudog/testudog.h"
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

int main(){
    robot::Testudog test;

    double body_front = 0.252;
    double body_back = 0.102;
    double body_width = 0.150;
    double hip_length = 0.0623;

    double x = 0;
    double y = 0;
    double z = 0.15;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double x_offset = 0.05;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x - body_width/2 - hip_length + x_offset;
    pos(1,0) = y + body_front;
    pos(2,0) = 0;
    //FR
    pos(0,1) = x + body_width/2 + hip_length + x_offset;
    pos(1,1) = y + body_front;
    pos(2,1) = 0;
    //BL
    pos(0,2) = x - body_width/2 - hip_length + x_offset;
    pos(1,2) = y - body_back;
    pos(2,2) = 0;
    //BR
    pos(0,3) = x + body_width/2 + hip_length + x_offset;
    pos(1,3) = y - body_back;
    pos(2,3) = 0;
    

    Eigen::Matrix<double, 3, 4> result = test.global2local_legpos(pos,x,y,z,roll,pitch,yaw);

    for (int i=0; i<=3; i++){
        std::cout << result(0,i) << '\n';
        std::cout << result(1,i) << '\n';
        std::cout << result(2,i) << '\n';
        std::cout << "................" << '\n';
    }

    std::cout << "##############" << '\n';
    Eigen::Matrix<double, 3, 4> result1 = test.inv_kine(result);

    for (int i=0; i<=3; i++){
        std::cout << result1(0,i) << '\n';
        std::cout << result1(1,i) << '\n';
        std::cout << result1(2,i) << '\n';
        std::cout << "................" << '\n';
    }

}
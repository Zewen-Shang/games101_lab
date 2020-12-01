#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    Eigen::Matrix3f rotate,trans;
    Eigen::Vector3f v(2,1,1);
    rotate << cos(45),-1*sin(45),0,
              sin(45),cos(45),0,
              0,0,1;
    trans << 1,0,1,
            0,1,2,
            0,0,1;
    std::cout<<trans*rotate*v;
    return 0;
}
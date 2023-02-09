#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include "../Inc/mekf_CPP.h"
#include "../CPP_libraries/eigen-3.4.0/Eigen/Dense"
#include "../CPP_libraries/eigen-3.4.0/unsupported/Eigen/MatrixFunctions"

//Disclaimer: I'm aware that this code is poorly optimized and does things in a very roundabout way.
//            I just hope it works. I don't have the energy to refactor.

MEKF::MEKF(double dt,double sigma_gyro, double sigma_acc, double sigma_gps, double P_init_orien, double P_init_pos) {
    //double
    this->T = dt;

    //vectors
    this->Va_k_1 = Eigen::Vector3d::Zero(); //3x1
    this->Va_k = Eigen::Vector3d::Zero(); //3x1
    this->ra_k_1 = Eigen::Vector3d::Zero(); //3x1
    this->ra_k = Eigen::Vector3d::Zero(); //3x1
    this->ga << 0.0, 
                0.0,
                -9.81; //3x1

    this->gyro = Eigen::Vector3d::Zero(); //3x1
    this->acc = Eigen::Vector3d::Zero(); //3x1
    this->gps = Eigen::Vector3d::Zero(); //3x1

    this->correction_term = Eigen::MatrixXd::Zero(9, 1); //9x1 zeros

    //static matrices
    this->Cab_k_1 = eulerToDcm(0.0, 0.0, 0.0); //3x3
    this->Cab_k = Eigen::MatrixXd::Identity(3, 3); //3x3

    //dynamic matrices
    this->A = Eigen::MatrixXd::Zero(9, 9); //9x9 zeros

    double cov_gyro = sigma_gyro * sigma_gyro;
    double cov_acc = sigma_acc * sigma_acc;
    double cov_gps = sigma_gps * sigma_gps;
    this->Q.resize(6, 6);
    this->Q <<  cov_gyro, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, cov_gyro, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, cov_gyro, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, cov_acc, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, cov_acc, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, cov_acc; //6x6 
    this->R = (Eigen::MatrixXd::Identity(3, 3).array() * cov_gps).matrix(); //3x3

    this->P_k = Eigen::MatrixXd::Identity(9, 9); //9x9
    this->P_k_1.resize(9, 9);
    this->P_k_1 << P_init_orien, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //could be done with diag matrices? 
                   0.0, P_init_orien, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                   0.0, 0.0, P_init_orien, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 0.0, 
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 0.0, 
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos, 0.0, 
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P_init_pos; //9x9
    this->L = Eigen::MatrixXd::Zero(9, 6); //9x6 
    this->S1 = Eigen::MatrixXd::Identity(9, 9); //9x9
    this->S2 = Eigen::MatrixXd::Identity(3, 3); //3x3

    this->K_k = Eigen::MatrixXd::Zero(9, 3); //9x3 
    this->M_k = Eigen::MatrixXd::Identity(3, 3); //3x3
    this->C_k.resize(3, 9);
    this->C_k << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; //3x9 

}

void MEKF::kf_predict(Eigen::Vector3d gyro_input, Eigen::Vector3d acc_input){
    Eigen::Matrix3d gyro_cross = crossOperator(gyro_input);
    Eigen::Matrix3d acc_cross = crossOperator(acc_input);
    this->Cab_k = this->Cab_k_1 * ((this->T * gyro_cross.array()).matrix()).exp();

    this->Va_k = this->Va_k_1 + (this->T * (this->Cab_k_1 * acc_input + this->ga).array()).matrix();
    this->ra_k = this->ra_k_1 + (this->T * this->Va_k_1.array()).matrix();

    Eigen::MatrixXd a1 = ((this->T * gyro_cross.array()).matrix()).exp(); //temp matrix with short name
    Eigen::MatrixXd a2 = a1.transpose();
    Eigen::MatrixXd b = (this->T * (this->Cab_k_1 * acc_cross).array()).matrix();
    Eigen::MatrixXd c = (Eigen::MatrixXd::Identity(3, 3).array() *this->T).matrix();
    this->A << a2(0, 0), a2(0, 1), a2(0, 2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //I wish I knew an easier way...
               a2(1, 0), a2(1, 1), a2(1, 2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               a2(2, 0), a2(2, 1), a2(2, 2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               b(0, 0), b(0, 1), b(0, 2), 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               b(1, 0), b(1, 1), b(1, 2), 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               b(2, 0), b(2, 1), b(2, 2), 1.0, 0.0, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, c(0, 0), c(0, 1), c(0, 2), 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, c(1, 0), c(1, 1), c(1, 2), 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, c(2, 0), c(2, 1), c(2, 2), 0.0, 0.0, 1.0;

    Eigen::MatrixXd d = (Eigen::MatrixXd::Identity(3, 3).array() * this->T).matrix();
    Eigen::MatrixXd e = (-this->T * this->Cab_k_1.array()).matrix();
    this->L << d(0, 0), d(0, 1), d(0, 2), 0.0, 0.0, 0.0,
               d(1, 0), d(1, 1), d(1, 2), 0.0, 0.0, 0.0,
               d(2, 0), d(2, 1), d(2, 2), 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, e(0, 0), e(0, 1), e(0, 2),
               0.0, 0.0, 0.0, e(1, 0), e(1, 1), e(1, 2),
               0.0, 0.0, 0.0, e(2, 0), e(2, 1), e(2, 2),
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    this->P_k = this->A * this->P_k_1 * this->A.transpose() + this->L * this->Q * this->L.transpose();
}

void MEKF::kf_correct(Eigen::Vector3d gps_input) {

    this->S2 = this->M_k  * this->R * this->M_k.transpose();
    this->K_k = this->P_k * this->C_k.transpose() * (this->C_k * this->P_k * this->C_k.transpose() + this->S2).inverse();
    Eigen::MatrixXd i9 = Eigen::MatrixXd::Identity(9, 9);
    this->S1 = i9 - this->K_k * this->C_k;
    this->P_k = this->S1 * this->P_k * this->S1.transpose() + this->K_k * this->S2 * this->K_k.transpose();

    this->correction_term = this->K_k * (gps_input - this->ra_k);

    Eigen::Vector3d f;
    f << this->correction_term(0),
               correction_term(1),
               correction_term(2);
    Eigen::Vector3d g;
    g <<  this->correction_term(3),
          this->correction_term(4),
          this->correction_term(5);
    Eigen::Vector3d h;
    h << this->correction_term(6),
         this->correction_term(7),
         this->correction_term(8);

    Eigen::Matrix3d correct_cross = crossOperator(f);
    this->Cab_k = this->Cab_k * (-correct_cross).exp();
    this->Va_k = this->Va_k + g;
    this->ra_k = this->ra_k + h;
}

void MEKF::kf_update() {

    this->Cab_k_1 = this->Cab_k;
    this->Va_k_1 = this->Va_k;
    this->ra_k_1 = this->ra_k;
    this->P_k_1 = this->P_k;
}

double* MEKF::getPosition() {
    static double position[3];
    double *ptr;
    ptr = &position[0]; 
    position[0] = this->ra_k(0);
    position[1] = this->ra_k(1);
    position[2] = this->ra_k(2);
    return ptr; //returning pointer to first element in array. Use ptr++ to access other elements.
}

//Helper methods 

//creates a rotation matrix from Euler angles 
Eigen::Matrix3d MEKF::eulerToDcm(double roll,double yaw, double pitch) {

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    return rotationMatrix;
}

Eigen::Vector3d MEKF::dcmToEuler(Eigen::Matrix3d dcm) {
    Eigen::Matrix3d c = dcm.log();
    Eigen::Vector3d d;
    d << c(2, 1), 
         c(0, 2), 
         c(1, 0);
    return d;
}

//applies the cross operator on a 3d vector
Eigen::MatrixXd MEKF::crossOperator(Eigen::Vector3d c) {
    Eigen::MatrixXd cross;
    cross.resize(3, 3);
    cross << 0.0, -c(2), c(1),
             c(2), 0.0, -c(0),
             -c(1), c(0), 0.0;
    return cross;
}
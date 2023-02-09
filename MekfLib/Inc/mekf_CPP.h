#ifndef MEKF_CPP_H_
#define MEKF_CPP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../CPP_libraries/eigen-3.4.0/Eigen/Dense"

class MEKF {

    public:
        MEKF (double dt,double sigma_gyro, double sigma_acc, double sigma_gps, double P_init_orien, double P_init_pos);

        //double
        double T = 0.1;

        //vectors
        Eigen::Vector3d Va_k_1;
        Eigen::Vector3d Va_k;
        Eigen::Vector3d ra_k_1;
        Eigen::Vector3d ra_k;
        Eigen::Vector3d ga; //3x1

        Eigen::Vector3d gyro; //3x1
        Eigen::Vector3d acc; //3x1
        Eigen::Vector3d gps; //3x1

        Eigen::VectorXd correction_term; //9x1 zeros

        //static matrices
        Eigen::Matrix3d Cab_k_1;
        Eigen::Matrix3d Cab_k;

        //dynamic matrices
        Eigen::MatrixXd A; //9x9 zeros
        Eigen::MatrixXd Q; //6x6 
        Eigen::MatrixXd R; //3x3

        Eigen::MatrixXd P_k; //9x9
        Eigen::MatrixXd P_k_1;
        Eigen::MatrixXd L; //9x6 
        Eigen::MatrixXd S1;  
        Eigen::MatrixXd S2;

        Eigen::MatrixXd K_k; //9x3 
        Eigen::MatrixXd M_k; //3x3
        Eigen::MatrixXd C_k; //3x9 

        //predict step
        void kf_predict(Eigen::Vector3d gyro_input, Eigen::Vector3d acc_input);

        //correct step
        void kf_correct(Eigen::Vector3d gps_input);

        // t -> t+1
        void kf_update();

        //getter 
        double* getPosition();

        //Helpers 
        Eigen::Matrix3d eulerToDcm(double roll, double yaw, double pitch);

        Eigen::Vector3d dcmToEuler(Eigen::Matrix3d dcm);

        Eigen::MatrixXd crossOperator(Eigen::Vector3d vector);
};
#endif 
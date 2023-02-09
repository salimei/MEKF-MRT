#include "../Inc/mekf_CPP.h"
#include "../Inc/wrapper.h"

extern "C" {
        MEKF* newMEKF(double dt,double sigma_gyro, double sigma_acc, double sigma_gps, double P_init_orien, double P_init_pos) {
                return new MEKF(dt, sigma_gyro, sigma_acc, sigma_gps, P_init_orien, P_init_pos);
        }

        void MEKF_predict(MEKF* v, double gyro_input[], double acc_input[]) {
            Eigen::Vector3d gyro_input_E;
            gyro_input_E << gyro_input[0],
                            gyro_input[1],
                            gyro_input[2];
            Eigen::Vector3d acc_input_E;
            acc_input_E << acc_input[0],
                           acc_input[1],
                           acc_input[2];
            v->kf_predict(gyro_input_E, acc_input_E);
        }

        void MEKF_correct(MEKF* v, double gps_input[]) {
            Eigen::Vector3d gps_input_E;
            gps_input_E << gps_input[0],
                           gps_input[1],
                           gps_input[2];
            v->kf_correct(gps_input_E);
        }

        void MEKF_update(MEKF* v) {
            v->kf_update();
        }

        double* MEKF_getPosition(MEKF* v) {
            double *a = v->getPosition();
            return a;
        }

        void deleteMEKF(MEKF* v) {
                delete v;
        }
}
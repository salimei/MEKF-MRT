#ifndef __WRAPPER_H
#define __WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MEKF MEKF;

MEKF* newMEKF(double dt,double sigma_gyro, double sigma_acc, double sigma_gps, double P_init_orien, double P_init_pos); //constructor

void MEKF_predict(MEKF* v, double gyro_input[], double acc_input[]);//predict

void MEKF_correct(MEKF* v, double gps_input[]);//correct

void MEKF_update(MEKF* v);//update

double* MEKF_getPosition(MEKF* v); //getter

void deleteMEKF(MEKF* v); //destroyer

#ifdef __cplusplus
}
#endif
#endif
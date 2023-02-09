#include "../Inc/wrapper.h"
#include <stdio.h>

int main(int argc, char* argv[]) {
        struct MEKF* c = newMEKF(0.01, 0.1, 0.1, 0.5, 0.1, 0.5);
        	
        double gyro_input[3] = { 0.0, 0.0, 0.0 }; 
        double acc_input[3] = {1.0, 0.0, 0.0};
        double *p;

        for(int i=0; i<100; i++) {
            MEKF_predict(c, gyro_input, acc_input);
            MEKF_update(c);

            p = MEKF_getPosition(c);
            printf("%.5f\t", *p);
            printf("%.5f\t", *(++p));
            printf("%.5f\t", *(++p));
            printf("\n");
        }

        deleteMEKF(c);
}
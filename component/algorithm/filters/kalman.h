//
// Created by Henson on 2024-01-17.
//

#ifndef __KALMAN_H
#define __KALMAN_H

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    /* Kalman filter variables */
    double Q_angle; //Process noise variance for the accelerometer
    double Q_bias;  //Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; //P¾ØÕó Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
} KALMAN;

typedef struct{
    /* Kalman filter variables */
    float x;
    float A;
    float B;
    float H;
    float Q; //Process noise variance for the accelerometer
    float R; // Measurement noise variance - this is actually the variance of the measurement noise
    float P; // Error covariance matrix - This is a 2x2 matrix
    float K; // Kalman gain - This is a 2x1 vector
} KALMAN_1;

void KalmanFilter_Init(KALMAN_1 *);
float KalmanFilter_Update(KALMAN_1 *, float,float);


void KalmanInit(KALMAN* klm);
void KalmanCalc(KALMAN* klm, double NewAngle, double NewRate, double dt);
void SetAngle(KALMAN* klm,double newAngle);

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

#endif //__KALMAN_H

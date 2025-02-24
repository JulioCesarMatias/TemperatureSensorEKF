#include <Arduino.h>
#include "EKF.h"

// Prediction step of the EKF
void ExtendedKalmanFilter::prediction(float dt)
{
    // State transition matrix
    float A[4][4] = {{1, dt, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    // Process noise covariance
    float Q[4][4] = {{0.01f, 0, 0, 0}, {0, 0.01f, 0, 0}, {0, 0, 0.01f, 0}, {0, 0, 0, 0.01f}};

    // Predict the next state
    float xPred[3] = {
        temperature + dt * rate_of_change,
        rate_of_change,
        sensor_bias};

    // Predict the covariance matrix
    float AP[4][4], PCovPred[4][4];

    // matrix multiplication (4x4)
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            AP[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                AP[i][j] += A[i][k] * P[k][j];
            }
        }
    }

    // matrix multiplication (4x4)
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            PCovPred[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                PCovPred[i][j] += AP[i][k] * A[k][j];
            }
        }
    }

    // Add process noise
    for (int i = 0; i < 4; i++)
    {
        PCovPred[i][i] += Q[i][i];
    }

    // Update the state and covariance
    temperature = xPred[0];
    rate_of_change = (temperature - previous_temperature) / dt;
    previous_temperature = temperature;
    sensor_bias = xPred[2];

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P[i][j] = PCovPred[i][j];
        }
    }
}

// Update step of the EKF
void ExtendedKalmanFilter::update(float z, float dt)
{
    // Measurement noise covariance
    float R[1][1] = {{0.1f}};

    // Innovation covariance
    float S[1][1];
    S[0][0] = P[0][0] + P[0][2] + R[0][0];

    // Calculate the inverse of the innovation covariance
    float invS[1][1];

    // Invert a 1x1 matrix
    if (S[0][0] != 0)
    {
        invS[0][0] = 1.0f / S[0][0];
    }

    // Calculate the Kalman gain
    float K[4];
    K[0] = (P[0][0] + P[0][2]) * invS[0][0];
    K[1] = (P[1][0] + P[1][2]) * invS[0][0] * dt;
    K[2] = (P[2][0] + P[2][2]) * invS[0][0];

    // Measurement residual
    float y = z - (temperature + sensor_bias);

    // Update the state
    temperature += K[0] * y;
    rate_of_change = (temperature - previous_temperature) / dt;
    previous_temperature = temperature;
    sensor_bias += K[2] * y;
}

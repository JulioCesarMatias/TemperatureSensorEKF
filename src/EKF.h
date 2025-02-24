#include <Arduino.h>

// Extended Kalman Filter (EKF) for a temperature sensor
// Author: Julio Cesar Matias

class ExtendedKalmanFilter
{
public:
    // Constructor
    ExtendedKalmanFilter() : temperature(25.0f), rate_of_change(0.0f), sensor_bias(0.5f), previous_temperature(25.0f)
    {
        // Initialize the covariance matrix as an identity matrix
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                P[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    // State variables
    float temperature;          // Estimated temperature in Celsius
    float rate_of_change;       // Estimated rate of temperature change in Celsius per second
    float sensor_bias;          // Estimated sensor bias
    float previous_temperature; // Previous temperature for rate of change calculation

    void prediction(float dt);
    void update(float z, float dt);

private:
    // Covariance matrix (4x4)
    float P[4][4];
};
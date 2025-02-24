#include <Arduino.h>
#include "EKF.h"

ExtendedKalmanFilter ekf;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  float dt = 1.0f; // this loop time is running in 1Hz defined by the delay(1000)

  // Simulated true temperature and noisy measurement
  float true_temp = 25.0f + 0.1f * (millis() / 1000.0f);     // simulated temperature measured for the sensor with initial temp in 25°Celsius
  float temp_noise = ((random(100) / 100.0f - 0.5f) * 0.5f); // simulated temperature noise in floating point
  float measurement = true_temp + temp_noise;

  // EKF prediction and update
  ekf.prediction(dt);
  ekf.update(measurement + ekf.sensor_bias, dt);

  // Output results to Serial
  Serial.printf("Time: %lu s, True Temp: %.2f, Estimated Temp: %.2f, Rate of Change: %.2f, Bias: %.2f\n",
                millis() / 1000, true_temp, ekf.temperature, ekf.rate_of_change, ekf.sensor_bias);

  delay(1000);
}

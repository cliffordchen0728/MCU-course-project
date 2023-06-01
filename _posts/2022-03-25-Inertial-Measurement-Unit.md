---
layout: post
title: Inertial Measurement Unit (IMU)
author: [clifford chen]
category: [Lecture]
tags: [jekyll, ai]
---
影片
<iframe width="347" height="618" src="https://www.youtube.com/embed/jBPgw7o4a5I" title="2023年5月25日" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
程式碼
float Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;

float angle = 0.0f; // Reset the angle
float bias = 0.0f; // Reset bias

float P[2][2] = {0.0, 0.0, 0.0, 0.0};
    
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};  
![](https://github.com/cliffordchen0728/MCU-course-project/blob/main/images/imu.jpg?raw=true?raw=true)

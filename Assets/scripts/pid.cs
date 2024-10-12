using UnityEngine;

public class PIDController
{
    // PID Constants
    public float kp = 1.0f;  // Proportional gain
    public float ki = 0.0f;  // Integral gain
    public float kd = 0.0f;  // Derivative gain

    // Error terms
    private float previousError = 0.0f;
    private float integral = 0.0f;

    // PID update for a single joint (angle control)
    public float Update(float currentValue, float targetValue, float deltaTime)
    {
        // Calculate error
        float error = targetValue - currentValue;

        // Proportional term
        float proportional = kp * error;

        // Integral term
        integral += error * deltaTime;
        float integralTerm = ki * integral;

        // Derivative term
        float derivative = (error - previousError) / deltaTime;
        float derivativeTerm = kd * derivative;

        // Update previous error
        previousError = error;

        // Return PID output
        return proportional + integralTerm + derivativeTerm;
    }
}

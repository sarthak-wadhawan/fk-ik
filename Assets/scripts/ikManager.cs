using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine;

public class InverseKinematics : MonoBehaviour
{
    public ForwardKinematics fk; // Reference to the Forward Kinematics script
    public Joints jointsScript;   // Reference to the Joints script
    public gameObject m_target;     // The moving target position (e.g., the ball)

    private float learningRate = 0.1f; // Learning rate for gradient descent
    private int maxIterations = 100;    // Maximum iterations for convergence
    private float tolerance = 0.01f;     // Error tolerance for convergence

    void Start()
    {
        if (fk == null)
        {
            fk = GetComponent<ForwardKinematics>();
        }
    }

    void Update()
    {
        // Continuously perform Inverse Kinematics to move towards the target
        PerformIK(jointsScript.GetJointAngles());
    }

    public void PerformIK(float[] initialAngles)
    {
        float[] jointAngles = (float[])initialAngles.Clone(); // Clone to avoid modifying original angles

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            // Calculate the current end effector position
            Matrix<double> T_final = fk.ForwardKinematicsCalculation(jointAngles, jointsScript.GetLinkLengths());
            Vector3 currentPos = fk.ExtractEndEffectorPosition(T_final);
            Vector3 targetPos = m_target.transform.position; // Use m_target to get the current position of the moving target

            // Calculate the error
            Vector3 error = targetPos - currentPos;

            // Check for convergence
            if (error.magnitude < tolerance)
            {
                Debug.Log($"Converged after {iteration} iterations");
                break;
            }

            // Calculate gradients for each joint angle
            float[] gradients = CalculateGradients(jointAngles, error);

            // Update joint angles using gradient descent
            for (int i = 0; i < jointAngles.Length; i++)
            {
                jointAngles[i] += learningRate * gradients[i];
            }
        }

        // Set the new joint angles
        jointsScript.SetJointAngles(jointAngles);
    }

    private float[] CalculateGradients(float[] angles, Vector3 error)
    {
        float[] gradients = new float[angles.Length];
        float delta = 0.001f; // Small delta for numerical differentiation

        for (int i = 0; i < angles.Length; i++)
        {
            // Calculate the original position
            Matrix<double> originalT = fk.ForwardKinematicsCalculation(angles, jointsScript.GetLinkLengths());
            Vector3 originalPos = fk.ExtractEndEffectorPosition(originalT);

            // Create a copy of angles and adjust one joint angle
            float[] newAngles = (float[])angles.Clone();
            newAngles[i] += delta;

            // Calculate the new position
            Matrix<double> newT = fk.ForwardKinematicsCalculation(newAngles, jointsScript.GetLinkLengths());
            Vector3 newPos = fk.ExtractEndEffectorPosition(newT);

            // Calculate the difference in position
            Vector3 positionChange = newPos - originalPos;

            // Compute the gradient as the change in error over the change in joint angle
            gradients[i] = Vector3.Dot(positionChange, error.normalized) / delta;
        }

        return gradients;
    }
}

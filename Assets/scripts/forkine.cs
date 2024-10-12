using UnityEngine;

public class forkine : MonoBehaviour
{
    public Joints jointsScript; // Reference to the Joints script
    public GameObject target;    // Reference to the target GameObject (e.g., the ball)
    private float[] linkLengths;

    private void Start()
    {
        // Ensure link lengths are initialized from the Joints script
        linkLengths = jointsScript.GetLinkLengths();
    }

    public static Matrix4x4 DHTransformation(float theta, float d, float a, float alpha)
    {
        // Convert angles to radians
        theta = Mathf.Deg2Rad * theta;
        alpha = Mathf.Deg2Rad * alpha;

        // Initialize identity matrix
        Matrix4x4 T = Matrix4x4.identity;

        // Create the transformation matrix using DH convention
        T[0, 0] = Mathf.Cos(theta);
        T[0, 1] = -Mathf.Sin(theta) * Mathf.Cos(alpha);
        T[0, 2] = Mathf.Sin(theta) * Mathf.Sin(alpha);
        T[0, 3] = a * Mathf.Cos(theta);

        T[1, 0] = Mathf.Sin(theta);
        T[1, 1] = Mathf.Cos(theta) * Mathf.Cos(alpha);
        T[1, 2] = -Mathf.Cos(theta) * Mathf.Sin(alpha);
        T[1, 3] = a * Mathf.Sin(theta);

        T[2, 0] = 0;
        T[2, 1] = Mathf.Sin(alpha);
        T[2, 2] = Mathf.Cos(alpha);
        T[2, 3] = d;

        T[3, 0] = 0;
        T[3, 1] = 0;
        T[3, 2] = 0;
        T[3, 3] = 1;

        return T;
    }

    public Matrix4x4 ForwardKinematics(float[] angles, float[] lengths)
    {
        // Assuming the following DH parameters:
        // θ1 -> rotating_block_vertical_arm_joint
        // θ2 -> vertical_arm_forearm_joint
        // θ3 -> actuator_forearm_joint
        // θ4 -> left_gripper/right_gripper joint (end effector)

        // Extract joint angles (assumed in degrees)
        float theta1 = angles[0];
        float theta2 = angles[1];
        float theta3 = angles[2];
        float theta4 = angles[3];

        // Define DH parameters for each joint
        Matrix4x4 T1 = DHTransformation(theta1, lengths[0], 0, 90.0f);  // First transformation
        Matrix4x4 T2 = DHTransformation(theta2, 0.0f, lengths[1], 0.0f);   // Second transformation
        Matrix4x4 T3 = DHTransformation(theta3, 0.0f, lengths[2], 0.0f);   // Third transformation
        Matrix4x4 T4 = DHTransformation(theta4, 0.0f, lengths[3], 0.0f);   // Fourth transformation (end-effector rotation)

        // Chain the transformations
        Matrix4x4 T_final = T1 * T2 * T3 * T4;

        return T_final;
    }

    public Vector3 ExtractEndEffectorPosition(Matrix4x4 T)
    {
        float x = T.m03;  // x position
        float y = T.m13;  // y position
        float z = T.m23;  // z position

        return new Vector3(x, y, z);
    }

    public Quaternion ExtractEndEffectorRotation(Matrix4x4 T)
    {
        // Extract the rotation matrix (3x3 top-left corner)
        Vector3 forward = new Vector3(T.m02, T.m12, T.m22);
        Vector3 upwards = new Vector3(T.m01, T.m11, T.m21);

        // Convert the rotation matrix to a Quaternion
        return Quaternion.LookRotation(forward, upwards);
    }

    // Use this method to get the target position whenever needed
    public Vector3 GetTargetPosition()
    {
        if (target != null)
        {
            return target.transform.position;
        }
        else
        {
            Debug.LogWarning("Target GameObject is not assigned.");
            return Vector3.zero; // Return a default value if no target is assigned
        }
    }
}

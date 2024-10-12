using UnityEngine;

public class Joints : MonoBehaviour
{
    public ArticulationBody[] joints; // Array of joints in the robot arm

    void Start()
    {
        // Initialize joints if not manually set in Unity Inspector
        if (joints == null || joints.Length == 0)
        {
            joints = GetComponentsInChildren<ArticulationBody>();
        }
    }

    // Get joint angles
    public float[] GetJointAngles()
    {
        float[] jointAngles = new float[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            jointAngles[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg; // Get the joint angle in degrees
        }

        return jointAngles;
    }

    // Example to update joint angles externally if needed
    public void SetJointAngles(float[] angles)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            // Set joint target position using articulation body
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = angles[i];
            joints[i].xDrive = drive;
        }
    }
}

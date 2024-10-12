using UnityEngine;

public class Joints : MonoBehaviour
{
    public ArticulationBody[] joints; // Array of joints in the robot arm
    public Transform[] linkTransforms; // Array of the transforms corresponding to the links

    void Start()
    {
        // Initialize joints if not manually set in Unity Inspector
        if (joints == null || joints.Length == 0)
        {
            joints = GetComponentsInChildren<ArticulationBody>();
        }

        // Initialize link transforms if not manually set
        if (linkTransforms == null || linkTransforms.Length == 0)
        {
            linkTransforms = new Transform[joints.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                linkTransforms[i] = joints[i].transform;
            }
        }
    }

    // Get joint angles in degrees
    public float[] GetJointAngles()
    {
        float[] jointAngles = new float[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            jointAngles[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg; // Get the joint angle in degrees
        }

        return jointAngles;
    }

    // Get link lengths by calculating the distance between successive joint positions
    public float[] GetLinkLengths()
    {
        float[] linkLengths = new float[linkTransforms.Length - 1];

        for (int i = 0; i < linkTransforms.Length - 1; i++)
        {
            // Calculate the distance between successive joints
            linkLengths[i] = Vector3.Distance(linkTransforms[i].position, linkTransforms[i + 1].position);
        }

        return linkLengths;
    }

    // Get the end effector's angle in the world space (relative to the base or ground)
    public float GetEndEffectorAngle()
    {
        // Assuming the last joint corresponds to the end effector
        Transform endEffectorTransform = linkTransforms[linkTransforms.Length - 1];

        // Get the rotation of the end effector in degrees (you can choose a specific axis if needed)
        return endEffectorTransform.eulerAngles.z; // You can change to x or y axis based on the robot setup
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
